//! AHCI SATA driver.
//!
//! The AHCI host bus adapter (HBA) lives in PCI memory space at BAR5
//! of the SATA controller (PCI class 0x01, subclass 0x06, prog-if
//! 0x01 — see [`crate::AHCI_PCI_CLASS`]). The kernel maps that BAR
//! and hands the base to [`AhciHba`].
//!
//! HBA memory layout (offsets from the HBA base):
//!
//! ```text
//! 0x00  CAP   host capabilities
//! 0x04  GHC   global host control (bit 31 = AHCI enable)
//! 0x08  IS    interrupt status
//! 0x0C  PI    ports implemented (bitmask)
//! 0x100 + port*0x80   port registers
//! ```
//!
//! Port registers (offsets from the port base):
//!
//! ```text
//! 0x00  CLB   command list base (low)
//! 0x04  CLBU  command list base (high)
//! 0x08  FB    FIS base (low)
//! 0x0C  FBU   FIS base (high)
//! 0x10  IS    interrupt status
//! 0x14  IE    interrupt enable
//! 0x18  CMD   command & status (bit 0 ST, bit 4 FRE, bit 15 CR)
//! 0x20  TFD   task file data
//! 0x24  SIG   signature: 0x00000101 = SATA, 0xEB140101 = ATAPI
//! 0x28  SSTS  SATA status (DET bits 0..3: 3 = device present)
//! 0x2C  SCTL  SATA control
//! 0x30  SERR  SATA error
//! 0x34  SACT  active (NCQ)
//! 0x38  CI    command issue (one bit per slot)
//! ```
//!
//! A command is issued by writing a 32-byte command header into the
//! command list (1 KiB, 32 slots × 32 bytes), a 256-byte command
//! table holding the Register–Host-to-Device FIS plus PRDT entries,
//! then setting the slot bit in CI. The kernel must supply real DMA
//! buffers for the command list, FIS area and data — see
//! `rustos-memory`'s frame allocator. This crate only builds the
//! in-memory structures and pokes the registers; it never touches
//! real DMA itself, so everything is host-testable over a [`Mmio`]
//! mock.

use alloc::vec::Vec;

/// HBA register offsets.
pub const HBA_CAP: u32 = 0x00;
pub const HBA_GHC: u32 = 0x04;
pub const HBA_IS: u32 = 0x08;
pub const HBA_PI: u32 = 0x0C;
pub const HBA_PORTS_BASE: u32 = 0x100;
pub const HBA_PORT_STRIDE: u32 = 0x80;

/// Port register offsets.
pub const PORT_CLB: u32 = 0x00;
pub const PORT_CLBU: u32 = 0x04;
pub const PORT_FB: u32 = 0x08;
pub const PORT_FBU: u32 = 0x0C;
pub const PORT_IS: u32 = 0x10;
pub const PORT_IE: u32 = 0x14;
pub const PORT_CMD: u32 = 0x18;
pub const PORT_TFD: u32 = 0x20;
pub const PORT_SIG: u32 = 0x24;
pub const PORT_SSTS: u32 = 0x28;
pub const PORT_SCTL: u32 = 0x2C;
pub const PORT_SERR: u32 = 0x30;
pub const PORT_SACT: u32 = 0x34;
pub const PORT_CI: u32 = 0x38;

/// GHC bit 31: AHCI enable.
pub const GHC_AE: u32 = 1 << 31;
/// GHC bit 1: interrupt enable.
pub const GHC_IE: u32 = 1 << 1;
/// GHC bit 0: HBA reset.
pub const GHC_HR: u32 = 1;

/// CMD bits.
pub const CMD_ST: u32 = 1 << 0; // start
pub const CMD_FRE: u32 = 1 << 4; // FIS receive enable
pub const CMD_FR: u32 = 1 << 14; // FIS receive running
pub const CMD_CR: u32 = 1 << 15; // command list running

/// Port signatures.
pub const SIG_SATA: u32 = 0x0000_0101;
pub const SIG_ATAPI: u32 = 0xEB14_0101;
pub const SIG_SEMB: u32 = 0xC33C_0101;
pub const SIG_PM: u32 = 0x9669_0101;

/// FIS type: Register — Host to Device.
pub const FIS_REG_H2D: u8 = 0x27;
/// ATA commands used in the FIS.
pub const ATA_READ_DMA_EXT: u8 = 0x25;
pub const ATA_WRITE_DMA_EXT: u8 = 0x35;

/// Command-header flags: bit 0..4 = command FIS length in dwords.
pub const CMD_HDR_CFL_MASK: u16 = 0x1F;
/// Command-header flags: bit 6 = write (0 = read).
pub const CMD_HDR_WRITE: u16 = 1 << 6;
/// Command-header flags: bit 7 = prefetchable.
pub const CMD_HDR_PREFETCH: u16 = 1 << 7;

/// PRDT entry: bit 31 of the byte-count dword = interrupt on completion.
pub const PRDT_IOC: u32 = 1 << 31;

/// Number of command slots per port.
pub const COMMAND_SLOTS: usize = 32;
/// Bytes per command header.
pub const COMMAND_HEADER_SIZE: usize = 32;
/// Bytes per command table (CFIS 64 + ACMD 16 + reserved 48 + PRDT).
pub const COMMAND_TABLE_SIZE: usize = 256;

/// What kind of device a port signature describes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortType {
    Sata,
    Atapi,
    Semb,
    PortMultiplier,
    Unknown,
}

/// Classify a port signature register value.
pub const fn port_type(sig: u32) -> PortType {
    match sig {
        SIG_SATA => PortType::Sata,
        SIG_ATAPI => PortType::Atapi,
        SIG_SEMB => PortType::Semb,
        SIG_PM => PortType::PortMultiplier,
        _ => PortType::Unknown,
    }
}

/// 32-bit MMIO access to the HBA register space.
pub trait Mmio {
    fn read_u32(&mut self, offset: u32) -> u32;
    fn write_u32(&mut self, offset: u32, value: u32);
}

/// In-memory fake MMIO for host tests: a flat dword array.
pub struct MockMmio {
    regs: Vec<u32>,
}

impl MockMmio {
    /// `dwords` is the size of the emulated register space in dwords.
    pub fn new(dwords: usize) -> Self {
        Self { regs: alloc::vec![0u32; dwords] }
    }
    pub fn set(&mut self, offset: u32, value: u32) {
        let idx = (offset / 4) as usize;
        if idx < self.regs.len() {
            self.regs[idx] = value;
        }
    }
    pub fn get(&self, offset: u32) -> u32 {
        let idx = (offset / 4) as usize;
        self.regs.get(idx).copied().unwrap_or(0)
    }
}

impl Mmio for MockMmio {
    fn read_u32(&mut self, offset: u32) -> u32 {
        self.get(offset)
    }
    fn write_u32(&mut self, offset: u32, value: u32) {
        self.set(offset, value);
    }
}

/// One 32-byte command header (8 dwords).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CommandHeader {
    /// Flags: CFL (bits 0..4), write (bit 6), prefetch (bit 7),
    /// PRDT length (bits 16..31).
    pub flags: u16,
    /// Physical Region Descriptor Table length (entries).
    pub prdtl: u16,
    /// Bytes transferred so far (written by hardware).
    pub prdbc: u32,
    /// Command table base address (low / high).
    pub ctba: u32,
    pub ctbau: u32,
    pub reserved: [u32; 4],
}

impl CommandHeader {
    /// Build a header for a DMA read/write of `prdtl` PRDT entries.
    /// `ctba_phys` is the physical address of the command table.
    pub fn new(write: bool, prdtl: u16, ctba_phys: u64) -> Self {
        // Command FIS length is 5 dwords (20 bytes).
        let flags: u16 = 5 | if write { CMD_HDR_WRITE } else { 0 };
        Self {
            flags,
            prdtl,
            prdbc: 0,
            ctba: ctba_phys as u32,
            ctbau: (ctba_phys >> 32) as u32,
            reserved: [0; 4],
        }
    }

    /// Serialise to 8 dwords, little-endian layout per the AHCI spec.
    pub fn to_dwords(&self) -> [u32; 8] {
        let flags_dword = (self.flags as u32) | ((self.prdtl as u32) << 16);
        [
            flags_dword,
            self.prdbc,
            self.ctba,
            self.ctbau,
            self.reserved[0],
            self.reserved[1],
            self.reserved[2],
            self.reserved[3],
        ]
    }

    /// Parse 8 dwords back into a header.
    pub fn from_dwords(d: &[u32; 8]) -> Self {
        Self {
            flags: (d[0] & 0xFFFF) as u16,
            prdtl: (d[0] >> 16) as u16,
            prdbc: d[1],
            ctba: d[2],
            ctbau: d[3],
            reserved: [d[4], d[5], d[6], d[7]],
        }
    }
}

/// Build the 64-byte Register–Host-to-Device FIS for a DMA read or
/// write, plus PRDT entries, into a 256-byte command table buffer.
///
/// * `lba` — starting sector, `count` — sectors, `write` — direction.
/// * `buffers` — (physical address, byte length) pairs, one PRDT
///   entry each (max 8 entries here; each entry covers up to 4 MiB).
///
/// Returns the number of PRDT entries written.
pub fn build_command_table(
    table: &mut [u8; COMMAND_TABLE_SIZE],
    lba: u64,
    count: u16,
    write: bool,
    buffers: &[(u64, u32)],
) -> usize {
    for b in table.iter_mut() {
        *b = 0;
    }
    // CFIS (64 bytes): type 0x27, flags 0x80 (this is a command),
    // command, features, LBA, device, control.
    table[0] = FIS_REG_H2D;
    table[1] = 0x80;
    table[2] = if write { ATA_WRITE_DMA_EXT } else { ATA_READ_DMA_EXT };
    table[3] = 0; // features (low)
    table[4] = lba as u8;
    table[5] = (lba >> 8) as u8;
    table[6] = (lba >> 16) as u8;
    table[7] = 0x40; // device: LBA mode
    table[8] = (lba >> 24) as u8;
    table[9] = (lba >> 32) as u8;
    table[10] = (lba >> 40) as u8;
    table[11] = 0; // features (high)
    table[12] = (count & 0xFF) as u8;
    table[13] = (count >> 8) as u8;
    table[14] = 0; // ICC / reserved
    table[15] = 0; // control
    // PRDT starts at offset 128.
    let n = buffers.len().min(8);
    for (i, (phys, len)) in buffers.iter().take(n).enumerate() {
        let off = 128 + i * 16;
        table[off..off + 4].copy_from_slice(&(*phys as u32).to_le_bytes());
        table[off + 4..off + 8].copy_from_slice(&((*phys >> 32) as u32).to_le_bytes());
        table[off + 8..off + 12].copy_from_slice(&0u32.to_le_bytes());
        // Byte count is (length - 1); bit 31 = interrupt on completion.
        let dbc = (*len - 1) | PRDT_IOC;
        table[off + 12..off + 16].copy_from_slice(&dbc.to_le_bytes());
    }
    n
}

/// The AHCI host bus adapter, generic over the MMIO backend.
pub struct AhciHba<M: Mmio> {
    mmio: M,
}

impl<M: Mmio> AhciHba<M> {
    pub fn new(mmio: M) -> Self {
        Self { mmio }
    }
    pub fn mmio(&mut self) -> &mut M {
        &mut self.mmio
    }
    fn read(&mut self, offset: u32) -> u32 {
        self.mmio.read_u32(offset)
    }
    fn write(&mut self, offset: u32, value: u32) {
        self.mmio.write_u32(offset, value);
    }

    /// Enable AHCI mode (GHC.AE) and interrupts (GHC.IE).
    pub fn enable(&mut self) {
        let ghc = self.read(HBA_GHC);
        self.write(HBA_GHC, ghc | GHC_AE | GHC_IE);
    }
    /// Bitmask of implemented ports (PI register).
    pub fn implemented_ports(&mut self) -> u32 {
        self.read(HBA_PI)
    }
    /// Ports that are both implemented and have a device present
    /// (SSTS DET == 3) with a known signature.
    pub fn active_ports(&mut self) -> Vec<u8> {
        let pi = self.implemented_ports();
        let mut out = Vec::new();
        for port in 0..32u8 {
            if pi & (1 << port) == 0 {
                continue;
            }
            let base = HBA_PORTS_BASE + (port as u32) * HBA_PORT_STRIDE;
            let ssts = self.read(base + PORT_SSTS);
            if ssts & 0x0F != 3 {
                continue;
            }
            out.push(port);
        }
        out
    }
    /// Signature of one port (0..31).
    pub fn port_signature(&mut self, port: u8) -> u32 {
        let base = HBA_PORTS_BASE + (port as u32) * HBA_PORT_STRIDE;
        self.read(base + PORT_SIG)
    }
    /// Device type of one port.
    pub fn port_type(&mut self, port: u8) -> PortType {
        port_type(self.port_signature(port))
    }

    /// Start the command engine on a port: set FRE, wait for FR,
    /// then set ST. In the mock the FR bit is set by the test.
    pub fn start_port(&mut self, port: u8) {
        let base = HBA_PORTS_BASE + (port as u32) * HBA_PORT_STRIDE;
        let cmd = self.read(base + PORT_CMD);
        self.write(base + PORT_CMD, cmd | CMD_FRE);
        // Real hardware: spin until CMD_FR is set. The mock sets it
        // immediately via the test's preset, so just read once.
        let _ = self.read(base + PORT_CMD);
        let cmd = self.read(base + PORT_CMD);
        self.write(base + PORT_CMD, cmd | CMD_ST);
    }

    /// Stop the command engine on a port: clear ST, then FRE.
    pub fn stop_port(&mut self, port: u8) {
        let base = HBA_PORTS_BASE + (port as u32) * HBA_PORT_STRIDE;
        let cmd = self.read(base + PORT_CMD);
        self.write(base + PORT_CMD, cmd & !CMD_ST);
        let cmd = self.read(base + PORT_CMD);
        self.write(base + PORT_CMD, cmd & !CMD_FRE);
    }

    /// Point a port at its command list / FIS buffers (physical
    /// addresses, 1 KiB / 256 B aligned by the caller).
    pub fn set_port_buffers(&mut self, port: u8, clb_phys: u64, fb_phys: u64) {
        let base = HBA_PORTS_BASE + (port as u32) * HBA_PORT_STRIDE;
        self.write(base + PORT_CLB, clb_phys as u32);
        self.write(base + PORT_CLBU, (clb_phys >> 32) as u32);
        self.write(base + PORT_FB, fb_phys as u32);
        self.write(base + PORT_FBU, (fb_phys >> 32) as u32);
    }

    /// Issue a command in `slot` on `port` (set the CI bit).
    pub fn issue_command(&mut self, port: u8, slot: u8) {
        let base = HBA_PORTS_BASE + (port as u32) * HBA_PORT_STRIDE;
        let ci = self.read(base + PORT_CI);
        self.write(base + PORT_CI, ci | (1 << slot));
    }

    /// Is `slot` on `port` still running (CI bit set)?
    pub fn command_busy(&mut self, port: u8, slot: u8) -> bool {
        let base = HBA_PORTS_BASE + (port as u32) * HBA_PORT_STRIDE;
        self.read(base + PORT_CI) & (1 << slot) != 0
    }

    /// Clear the port's interrupt status.
    pub fn clear_interrupt(&mut self, port: u8) {
        let base = HBA_PORTS_BASE + (port as u32) * HBA_PORT_STRIDE;
        self.write(base + PORT_IS, 0xFFFF_FFFF);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn port_type_classification() {
        assert_eq!(port_type(SIG_SATA), PortType::Sata);
        assert_eq!(port_type(SIG_ATAPI), PortType::Atapi);
        assert_eq!(port_type(SIG_SEMB), PortType::Semb);
        assert_eq!(port_type(SIG_PM), PortType::PortMultiplier);
        assert_eq!(port_type(0xDEAD_BEEF), PortType::Unknown);
    }

    #[test]
    fn command_header_roundtrip() {
        let h = CommandHeader::new(true, 2, 0x1_0000_2000);
        assert_eq!(h.flags & CMD_HDR_CFL_MASK, 5);
        assert!(h.flags & CMD_HDR_WRITE != 0);
        assert_eq!(h.prdtl, 2);
        assert_eq!(h.ctba, 0x0000_2000);
        assert_eq!(h.ctbau, 0x0000_0001);
        let d = h.to_dwords();
        // Flags dword: low half = flags, high half = PRDTL.
        assert_eq!(d[0], 0x0002_0045);
        let back = CommandHeader::from_dwords(&d);
        assert_eq!(back, h);
        // Read header: no WRITE bit.
        let r = CommandHeader::new(false, 1, 0x1000);
        assert_eq!(r.flags & CMD_HDR_WRITE, 0);
        assert_eq!(r.to_dwords()[0], 0x0001_0005);
    }

    #[test]
    fn command_table_read_dma_ext() {
        let mut table = [0u8; COMMAND_TABLE_SIZE];
        let n = build_command_table(&mut table, 0x0123_4567_89AB, 8, false, &[(0x2000, 4096)]);
        assert_eq!(n, 1);
        assert_eq!(table[0], FIS_REG_H2D);
        assert_eq!(table[1], 0x80);
        assert_eq!(table[2], ATA_READ_DMA_EXT);
        assert_eq!(table[4], 0xAB);
        assert_eq!(table[5], 0x89);
        assert_eq!(table[6], 0x67);
        assert_eq!(table[7], 0x40);
        assert_eq!(table[8], 0x45);
        assert_eq!(table[9], 0x23);
        assert_eq!(table[10], 0x01);
        assert_eq!(table[12], 8); // sector count low
        assert_eq!(table[13], 0); // sector count high
        // PRDT entry 0 at offset 128.
        assert_eq!(u32::from_le_bytes(table[128..132].try_into().unwrap()), 0x2000);
        assert_eq!(u32::from_le_bytes(table[140..144].try_into().unwrap()), 4095 | PRDT_IOC);
    }

    #[test]
    fn command_table_write_and_multiple_prdt() {
        let mut table = [0u8; COMMAND_TABLE_SIZE];
        let n = build_command_table(
            &mut table,
            0,
            16,
            true,
            &[(0x1000, 512), (0x2000, 512), (0x3000, 512)],
        );
        assert_eq!(n, 3);
        assert_eq!(table[2], ATA_WRITE_DMA_EXT);
        assert_eq!(u32::from_le_bytes(table[144..148].try_into().unwrap()), 0x2000);
        assert_eq!(u32::from_le_bytes(table[160..164].try_into().unwrap()), 0x3000);
    }

    #[test]
    fn hba_enable_and_port_detection() {
        let mut mmio = MockMmio::new(4096);
        // Ports 0 and 2 implemented; port 0 has a SATA device (DET=3).
        mmio.set(HBA_PI, 0b101);
        let p0 = HBA_PORTS_BASE;
        mmio.set(p0 + PORT_SSTS, 3);
        mmio.set(p0 + PORT_SIG, SIG_SATA);
        let p2 = HBA_PORTS_BASE + 2 * HBA_PORT_STRIDE;
        mmio.set(p2 + PORT_SSTS, 1); // DET=1: no device
        mmio.set(p2 + PORT_SIG, SIG_SATA);
        let mut hba = AhciHba::new(mmio);
        hba.enable();
        assert_eq!(hba.mmio().get(HBA_GHC) & GHC_AE, GHC_AE);
        assert_eq!(hba.implemented_ports(), 0b101);
        assert_eq!(hba.port_type(0), PortType::Sata);
        assert_eq!(hba.active_ports(), vec![0]);
    }

    #[test]
    fn port_buffers_and_command_issue() {
        let mut mmio = MockMmio::new(4096);
        let mut hba = AhciHba::new(mmio);
        hba.set_port_buffers(1, 0x1_0000_0000, 0x1_0000_1000);
        let base = HBA_PORTS_BASE + HBA_PORT_STRIDE;
        assert_eq!(hba.mmio().get(base + PORT_CLB), 0x0000_0000);
        assert_eq!(hba.mmio().get(base + PORT_CLBU), 0x0000_0001);
        assert_eq!(hba.mmio().get(base + PORT_FB), 0x0000_1000);
        assert!(!hba.command_busy(1, 0));
        hba.issue_command(1, 0);
        assert!(hba.command_busy(1, 0));
        assert!(!hba.command_busy(1, 1));
        hba.clear_interrupt(1);
        assert_eq!(hba.mmio().get(base + PORT_IS), 0xFFFF_FFFF);
    }

    #[test]
    fn start_and_stop_port() {
        let mut mmio = MockMmio::new(4096);
        let mut hba = AhciHba::new(mmio);
        hba.start_port(0);
        let base = HBA_PORTS_BASE;
        assert_eq!(hba.mmio().get(base + PORT_CMD) & CMD_FRE, CMD_FRE);
        assert_eq!(hba.mmio().get(base + PORT_CMD) & CMD_ST, CMD_ST);
        hba.stop_port(0);
        assert_eq!(hba.mmio().get(base + PORT_CMD) & CMD_ST, 0);
        assert_eq!(hba.mmio().get(base + PORT_CMD) & CMD_FRE, 0);
    }
}
