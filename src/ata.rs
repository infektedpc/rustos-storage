//! ATA/IDE PIO driver.
//!
//! Talks to the classic IDE controller on the primary (0x1F0) and
//! secondary (0x170) channels. Transfers are PIO (programmed I/O):
//! the CPU moves every 16-bit word through the data port. No DMA /
//! UDMA yet — for DMA use the AHCI driver ([`crate::ahci`]).
//!
//! Commands used: IDENTIFY (0xEC), READ SECTORS (0x20, LBA28) /
//! READ SECTORS EXT (0x24, LBA48), WRITE SECTORS (0x30) / EXT (0x34),
//! FLUSH CACHE (0xEA). One command per sector is issued — simple and
//! correct; multi-sector commands are a later optimisation.
//!
//! Register layout (offsets from the channel base):
//!
//! ```text
//! 0  data (16-bit)      4  LBA mid
//! 1  error / features   5  LBA high
//! 2  sector count       6  drive / head (LBA bits 24..27 for LBA28)
//! 3  LBA low            7  status (read) / command (write)
//! ```
//!
//! Status bits: BSY 0x80, DRDY 0x40, DRQ 0x08, ERR 0x01.

use crate::block::{BlockDevice, BlockError};
use crate::ports::PortIo;
use crate::SECTOR_SIZE;

/// Primary IDE channel: base 0x1F0, control 0x3F6.
pub const PRIMARY_BASE: u16 = 0x1F0;
/// Secondary IDE channel: base 0x170, control 0x376.
pub const SECONDARY_BASE: u16 = 0x170;
pub const PRIMARY_CONTROL: u16 = 0x3F6;
pub const SECONDARY_CONTROL: u16 = 0x376;

/// Status bits.
pub const STATUS_BSY: u8 = 0x80;
pub const STATUS_DRDY: u8 = 0x40;
pub const STATUS_DRQ: u8 = 0x08;
pub const STATUS_ERR: u8 = 0x01;

/// ATA commands.
pub const CMD_IDENTIFY: u8 = 0xEC;
pub const CMD_READ_SECTORS: u8 = 0x20;
pub const CMD_READ_SECTORS_EXT: u8 = 0x24;
pub const CMD_WRITE_SECTORS: u8 = 0x30;
pub const CMD_WRITE_SECTORS_EXT: u8 = 0x34;
pub const CMD_FLUSH_CACHE: u8 = 0xEA;

/// Maximum sectors addressable with LBA28 (2^28).
pub const LBA28_MAX: u64 = 1 << 28;

/// Which IDE channel a drive sits on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Channel {
    Primary,
    Secondary,
}

impl Channel {
    pub const fn base(self) -> u16 {
        match self {
            Channel::Primary => PRIMARY_BASE,
            Channel::Secondary => SECONDARY_BASE,
        }
    }
    pub const fn control(self) -> u16 {
        match self {
            Channel::Primary => PRIMARY_CONTROL,
            Channel::Secondary => SECONDARY_CONTROL,
        }
    }
}

/// Master (device 0) or slave (device 1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Drive {
    Master,
    Slave,
}

impl Drive {
    const fn bit(self) -> u8 {
        match self {
            Drive::Master => 0x00,
            Drive::Slave => 0x10,
        }
    }
}

/// Drive/head register value for an LBA28 access.
/// Bit 6 (0x40) = LBA mode, bit 5 = drive, bits 0..3 = LBA bits 24..27.
pub const fn lba28_drive_head(lba: u64, drive: Drive) -> u8 {
    0xE0 | drive.bit() | ((lba >> 24) as u8 & 0x0F)
}

/// Drive/head register value for an LBA48 access (no high LBA bits
/// in the register — they go in the LBA high registers instead).
pub const fn lba48_drive_head(drive: Drive) -> u8 {
    0x40 | drive.bit()
}

/// Parsed IDENTIFY DEVICE data (512 bytes = 256 words).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct IdentifyData {
    /// Serial number, byte-swapped into ASCII, trailing spaces kept.
    pub serial: [u8; 20],
    /// Firmware revision.
    pub firmware: [u8; 8],
    /// Model string.
    pub model: [u8; 40],
    /// Total sectors (LBA28, words 60..61).
    pub lba28_sectors: u32,
    /// Word 83 bit 10: LBA48 supported.
    pub lba48_supported: bool,
    /// Total sectors (LBA48, words 100..103).
    pub lba48_sectors: u64,
}

impl IdentifyData {
    /// Parse the 256 words returned by IDENTIFY. ATA strings are
    /// stored big-endian inside each 16-bit word, so each pair of
    /// bytes is swapped.
    pub fn parse(words: &[u16; 256]) -> Self {
        let mut serial = [0u8; 20];
        let mut firmware = [0u8; 8];
        let mut model = [0u8; 40];
        copy_swapped(&mut serial, &words[10..20]);
        copy_swapped(&mut firmware, &words[23..27]);
        copy_swapped(&mut model, &words[27..47]);
        let lba28_sectors = (words[60] as u32) | ((words[61] as u32) << 16);
        let lba48_supported = words[83] & (1 << 10) != 0;
        let lba48_sectors = (words[100] as u64)
            | ((words[101] as u64) << 16)
            | ((words[102] as u64) << 32)
            | ((words[103] as u64) << 48);
        Self { serial, firmware, model, lba28_sectors, lba48_supported, lba48_sectors }
    }

    /// Total addressable sectors, preferring LBA48 when supported.
    pub fn total_sectors(&self) -> u64 {
        if self.lba48_supported && self.lba48_sectors != 0 {
            self.lba48_sectors
        } else {
            self.lba28_sectors as u64
        }
    }

    pub fn serial_str(&self) -> &str {
        core::str::from_utf8(&self.serial).unwrap_or("").trim_end()
    }
    pub fn model_str(&self) -> &str {
        core::str::from_utf8(&self.model).unwrap_or("").trim_end()
    }
}

fn copy_swapped(out: &mut [u8], words: &[u16]) {
    for (i, w) in words.iter().enumerate() {
        out[i * 2] = (w >> 8) as u8;
        out[i * 2 + 1] = *w as u8;
    }
}

/// An ATA drive on one channel, generic over the port I/O backend.
pub struct AtaDisk<P: PortIo> {
    ports: P,
    channel: Channel,
    drive: Drive,
    /// Set by [`AtaDisk::identify`]; 0 until then.
    sectors: u64,
    /// Set by `identify` from word 83 bit 10.
    lba48: bool,
}

impl<P: PortIo> AtaDisk<P> {
    pub fn new(ports: P, channel: Channel, drive: Drive) -> Self {
        Self { ports, channel, drive, sectors: 0, lba48: false }
    }
    pub fn channel(&self) -> Channel {
        self.channel
    }
    pub fn drive(&self) -> Drive {
        self.drive
    }
    /// Sectors reported by IDENTIFY (0 until `identify` runs).
    pub fn sectors(&self) -> u64 {
        self.sectors
    }
    pub fn uses_lba48(&self) -> bool {
        self.lba48
    }
    /// Access to the port backend (used by tests to inspect the log).
    pub fn ports_ref(&mut self) -> &mut P {
        &mut self.ports
    }

    fn reg(&self, offset: u16) -> u16 {
        self.channel.base() + offset
    }
    fn status(&mut self) -> u8 {
        self.ports.read_u8(self.reg(7))
    }
    fn select_drive(&mut self, lba_high: u8) {
        // Drive/head register; LBA bit set, high LBA bits for LBA28.
        self.ports.write_u8(self.reg(6), 0xE0 | self.drive.bit() | (lba_high & 0x0F));
    }

    /// Wait until BSY clears (bounded). Returns `Timeout` if the
    /// drive stays busy, `DeviceError` if it reports ERR.
    fn wait_not_busy(&mut self) -> Result<(), BlockError> {
        for _ in 0..100_000 {
            let s = self.status();
            if s & STATUS_BSY == 0 {
                if s & STATUS_ERR != 0 {
                    return Err(BlockError::DeviceError);
                }
                return Ok(());
            }
        }
        Err(BlockError::Timeout)
    }

    /// Wait until BSY clears and DRQ is set (data ready to transfer).
    fn wait_drq(&mut self) -> Result<(), BlockError> {
        for _ in 0..100_000 {
            let s = self.status();
            if s & STATUS_BSY == 0 {
                if s & STATUS_ERR != 0 {
                    return Err(BlockError::DeviceError);
                }
                if s & STATUS_DRQ != 0 {
                    return Ok(());
                }
            }
        }
        Err(BlockError::Timeout)
    }

    /// Run IDENTIFY DEVICE. On success fills `sectors` / `lba48`.
    /// Returns `DeviceError` when no drive answers (status reads 0).
    pub fn identify(&mut self) -> Result<IdentifyData, BlockError> {
        self.select_drive(0);
        // Clear the other registers (the spec's recommended setup).
        self.ports.write_u8(self.reg(2), 0);
        self.ports.write_u8(self.reg(3), 0);
        self.ports.write_u8(self.reg(4), 0);
        self.ports.write_u8(self.reg(5), 0);
        self.ports.write_u8(self.reg(7), CMD_IDENTIFY);
        if self.status() == 0 {
            return Err(BlockError::DeviceError);
        }
        self.wait_drq()?;
        let mut words = [0u16; 256];
        for w in words.iter_mut() {
            *w = self.ports.read_u16(self.reg(0));
        }
        let info = IdentifyData::parse(&words);
        self.sectors = info.total_sectors();
        self.lba48 = info.lba48_supported;
        Ok(info)
    }

    /// Program the LBA registers for one sector and issue `command`.
    /// Uses LBA48 when the address needs it (or the drive was
    /// identified as LBA48-only — LBA28 addresses still use the
    /// shorter form when they fit).
    fn program_lba(&mut self, lba: u64, command28: u8, command48: u8) {
        if lba < LBA28_MAX {
            self.ports.write_u8(self.reg(2), 1); // sector count = 1
            self.ports.write_u8(self.reg(3), lba as u8);
            self.ports.write_u8(self.reg(4), (lba >> 8) as u8);
            self.ports.write_u8(self.reg(5), (lba >> 16) as u8);
            self.ports.write_u8(self.reg(6), lba28_drive_head(lba, self.drive));
            self.ports.write_u8(self.reg(7), command28);
        } else {
            // LBA48: high bytes first, then low bytes.
            self.ports.write_u8(self.reg(2), 0); // count high = 0
            self.ports.write_u8(self.reg(3), (lba >> 24) as u8);
            self.ports.write_u8(self.reg(4), (lba >> 32) as u8);
            self.ports.write_u8(self.reg(5), (lba >> 40) as u8);
            self.ports.write_u8(self.reg(2), 1); // count low = 1
            self.ports.write_u8(self.reg(3), lba as u8);
            self.ports.write_u8(self.reg(4), (lba >> 8) as u8);
            self.ports.write_u8(self.reg(5), (lba >> 16) as u8);
            self.ports.write_u8(self.reg(6), lba48_drive_head(self.drive));
            self.ports.write_u8(self.reg(7), command48);
        }
    }

    fn read_one_sector(&mut self, lba: u64, out: &mut [u8]) -> Result<(), BlockError> {
        self.program_lba(lba, CMD_READ_SECTORS, CMD_READ_SECTORS_EXT);
        self.wait_drq()?;
        for chunk in out.chunks_exact_mut(2) {
            let w = self.ports.read_u16(self.reg(0));
            chunk[0] = w as u8;
            chunk[1] = (w >> 8) as u8;
        }
        Ok(())
    }

    fn write_one_sector(&mut self, lba: u64, data: &[u8]) -> Result<(), BlockError> {
        self.program_lba(lba, CMD_WRITE_SECTORS, CMD_WRITE_SECTORS_EXT);
        self.wait_drq()?;
        for chunk in data.chunks_exact(2) {
            let w = (chunk[0] as u16) | ((chunk[1] as u16) << 8);
            self.ports.write_u16(self.reg(0), w);
        }
        Ok(())
    }

    /// Issue FLUSH CACHE (0xEA) and wait for the drive to go idle.
    pub fn flush_cache(&mut self) -> Result<(), BlockError> {
        self.ports.write_u8(self.reg(7), CMD_FLUSH_CACHE);
        self.wait_not_busy()
    }
}

impl<P: PortIo> BlockDevice for AtaDisk<P> {
    fn sector_size(&self) -> usize {
        SECTOR_SIZE
    }
    fn sector_count(&self) -> u64 {
        self.sectors
    }
    fn read_sectors(&mut self, lba: u64, buf: &mut [u8]) -> Result<(), BlockError> {
        if buf.len() % SECTOR_SIZE != 0 {
            return Err(BlockError::BadBuffer);
        }
        let count = (buf.len() / SECTOR_SIZE) as u64;
        if self.sectors != 0 && lba.checked_add(count).map_or(true, |e| e > self.sectors) {
            return Err(BlockError::OutOfRange);
        }
        for i in 0..count {
            let start = (i as usize) * SECTOR_SIZE;
            self.read_one_sector(lba + i, &mut buf[start..start + SECTOR_SIZE])?;
        }
        Ok(())
    }
    fn write_sectors(&mut self, lba: u64, buf: &[u8]) -> Result<(), BlockError> {
        if buf.len() % SECTOR_SIZE != 0 {
            return Err(BlockError::BadBuffer);
        }
        let count = (buf.len() / SECTOR_SIZE) as u64;
        if self.sectors != 0 && lba.checked_add(count).map_or(true, |e| e > self.sectors) {
            return Err(BlockError::OutOfRange);
        }
        for i in 0..count {
            let start = (i as usize) * SECTOR_SIZE;
            self.write_one_sector(lba + i, &buf[start..start + SECTOR_SIZE])?;
        }
        Ok(())
    }
    fn flush(&mut self) -> Result<(), BlockError> {
        self.flush_cache()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ports::MockPorts;

    /// Craft IDENTIFY words: model "QEMU HARDDISK", serial "QM00001",
    /// 2_000_000 LBA28 sectors, LBA48 supported with 4_000_000 sectors.
    fn identify_words() -> [u16; 256] {
        let mut w = [0u16; 256];
        // Fill a word field, padding with spaces to its full width.
        let put = |w: &mut [u16; 256], start: usize, count_words: usize, s: &str| {
            let bytes = s.as_bytes();
            for i in 0..count_words {
                let hi = bytes.get(i * 2).copied().unwrap_or(b' ');
                let lo = bytes.get(i * 2 + 1).copied().unwrap_or(b' ');
                w[start + i] = ((hi as u16) << 8) | lo as u16;
            }
        };
        put(&mut w, 10, 10, "QM00001");
        put(&mut w, 23, 4, "1.0");
        put(&mut w, 27, 20, "QEMU HARDDISK");
        w[60] = (2_000_000u32 & 0xFFFF) as u16;
        w[61] = (2_000_000u32 >> 16) as u16;
        w[83] = 1 << 10; // LBA48 supported
        w[100] = (4_000_000u64 & 0xFFFF) as u16;
        w[101] = ((4_000_000u64 >> 16) & 0xFFFF) as u16;
        w
    }

    #[test]
    fn drive_head_encoding() {
        assert_eq!(lba28_drive_head(0x0123_4567, Drive::Master), 0xE1);
        assert_eq!(lba28_drive_head(0x0123_4567, Drive::Slave), 0xF1);
        assert_eq!(lba28_drive_head(0, Drive::Master), 0xE0);
        assert_eq!(lba48_drive_head(Drive::Master), 0x40);
        assert_eq!(lba48_drive_head(Drive::Slave), 0x50);
    }

    #[test]
    fn identify_parsing() {
        let info = IdentifyData::parse(&identify_words());
        assert_eq!(info.serial_str(), "QM00001");
        assert_eq!(info.model_str(), "QEMU HARDDISK");
        assert_eq!(info.lba28_sectors, 2_000_000);
        assert!(info.lba48_supported);
        assert_eq!(info.lba48_sectors, 4_000_000);
        assert_eq!(info.total_sectors(), 4_000_000);
        // Without LBA48, LBA28 wins.
        let mut w = identify_words();
        w[83] = 0;
        let info = IdentifyData::parse(&w);
        assert!(!info.lba48_supported);
        assert_eq!(info.total_sectors(), 2_000_000);
    }

    #[test]
    fn identify_via_mock_ports() {
        let mut ports = MockPorts::new();
        ports.set(0x1F7, STATUS_DRDY | STATUS_DRQ); // ready + data ready
        let words = identify_words();
        ports.feed_data_words(&words);
        let mut disk = AtaDisk::new(ports, Channel::Primary, Drive::Master);
        let info = disk.identify().unwrap();
        assert_eq!(info.model_str(), "QEMU HARDDISK");
        assert_eq!(disk.sectors(), 4_000_000);
        assert!(disk.uses_lba48());
    }

    #[test]
    fn identify_no_drive() {
        let mut ports = MockPorts::new();
        ports.set(0x1F7, 0); // status reads 0: nothing there
        let mut disk = AtaDisk::new(ports, Channel::Primary, Drive::Master);
        assert_eq!(disk.identify(), Err(BlockError::DeviceError));
    }

    #[test]
    fn read_sector_lba28_via_mock() {
        let mut ports = MockPorts::new();
        ports.set(0x1F7, STATUS_DRDY | STATUS_DRQ);
        // One sector of data: word i == i.
        let words: Vec<u16> = (0..256u16).collect();
        ports.feed_data_words(&words);
        let mut disk = AtaDisk::new(ports, Channel::Primary, Drive::Master);
        let mut buf = [0u8; 512];
        disk.read_sectors(0x123, &mut buf).unwrap();
        // Word 5 == 5 -> bytes [5, 0].
        assert_eq!(&buf[10..12], &[5, 0]);
        assert_eq!(&buf[510..512], &[255, 0]);
    }

    #[test]
    fn read_issues_lba28_command_and_registers() {
        let mut ports = MockPorts::new();
        ports.set(0x1F7, STATUS_DRDY | STATUS_DRQ);
        ports.feed_data_words(&[0u16; 256]);
        let mut disk = AtaDisk::new(ports, Channel::Primary, Drive::Master);
        let mut buf = [0u8; 512];
        disk.read_sectors(0x0123_4567, &mut buf).unwrap();
        // The last write to the command register (0x1F7) is READ_SECTORS.
        let mut written = [0u8; 16];
        let n = disk
            .ports_ref()
            .bytes_written_to(0x1F7, &mut written);
        assert_eq!(written[n - 1], CMD_READ_SECTORS);
        // LBA bytes: low 0x67, mid 0x45, high 0x23.
        assert_eq!(disk.ports_ref().get(0x1F3), 0x67);
        assert_eq!(disk.ports_ref().get(0x1F4), 0x45);
        assert_eq!(disk.ports_ref().get(0x1F5), 0x23);
        // Drive/head: 0xE0 | LBA bits 24..27 (0x01).
        assert_eq!(disk.ports_ref().get(0x1F6), 0xE1);
    }

    #[test]
    fn read_large_lba_uses_lba48() {
        let mut ports = MockPorts::new();
        ports.set(0x1F7, STATUS_DRDY | STATUS_DRQ);
        ports.feed_data_words(&[0u16; 256]);
        let mut disk = AtaDisk::new(ports, Channel::Primary, Drive::Master);
        let mut buf = [0u8; 512];
        // 2^28 + 0x12345: needs LBA48.
        disk.read_sectors(LBA28_MAX + 0x12345, &mut buf).unwrap();
        let mut written = [0u8; 16];
        let n = disk.ports_ref().bytes_written_to(0x1F7, &mut written);
        assert_eq!(written[n - 1], CMD_READ_SECTORS_EXT);
        assert_eq!(disk.ports_ref().get(0x1F6), 0x40); // LBA48 drive/head
    }

    #[test]
    fn write_sector_captures_data_words() {
        let mut ports = MockPorts::new();
        ports.set(0x1F7, STATUS_DRDY | STATUS_DRQ);
        let mut disk = AtaDisk::new(ports, Channel::Primary, Drive::Master);
        let mut buf = [0u8; 512];
        buf[0] = 0x34;
        buf[1] = 0x12;
        buf[2] = 0xCD;
        buf[3] = 0xAB;
        disk.write_sectors(0, &buf).unwrap();
        let written = disk.ports_ref().data_written();
        assert_eq!(written.len(), 256);
        assert_eq!(written[0], 0x1234);
        assert_eq!(written[1], 0xABCD);
        let mut cmd = [0u8; 16];
        let n = disk.ports_ref().bytes_written_to(0x1F7, &mut cmd);
        assert_eq!(cmd[n - 1], CMD_WRITE_SECTORS);
    }

    #[test]
    fn busy_drive_times_out() {
        let mut ports = MockPorts::new();
        ports.set(0x1F7, STATUS_BSY); // never clears
        let mut disk = AtaDisk::new(ports, Channel::Primary, Drive::Master);
        let mut buf = [0u8; 512];
        assert_eq!(disk.read_sectors(0, &mut buf), Err(BlockError::Timeout));
    }

    #[test]
    fn error_bit_is_device_error() {
        let mut ports = MockPorts::new();
        ports.set(0x1F7, STATUS_ERR);
        let mut disk = AtaDisk::new(ports, Channel::Primary, Drive::Master);
        let mut buf = [0u8; 512];
        assert_eq!(disk.read_sectors(0, &mut buf), Err(BlockError::DeviceError));
    }

    #[test]
    fn block_device_bounds() {
        let mut ports = MockPorts::new();
        ports.set(0x1F7, STATUS_DRDY | STATUS_DRQ);
        ports.feed_data_words(&[0u16; 512]);
        let mut disk = AtaDisk::new(ports, Channel::Primary, Drive::Master);
        // sectors == 0 until identify: no range check, transfers work.
        let mut buf = [0u8; 1024];
        assert_eq!(disk.read_sectors(0, &mut buf), Ok(()));
        assert_eq!(disk.read_sectors(0, &mut [0u8; 100]), Err(BlockError::BadBuffer));
    }
}
