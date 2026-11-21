//! The block-device abstraction every storage driver implements,
//! plus an in-memory mock for host tests.

use crate::SECTOR_SIZE;
use alloc::vec::Vec;

/// Why a block operation failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlockError {
    /// `lba + count` runs past the end of the device.
    OutOfRange,
    /// The device reported an error (ATA ERR bit, AHCI error, …).
    DeviceError,
    /// A status poll never reached the expected state.
    Timeout,
    /// The caller's buffer is the wrong size for the request.
    BadBuffer,
    /// Injected failure (mock only).
    InjectedFailure,
}

/// A sector-addressed storage device. One sector is 512 bytes
/// ([`crate::SECTOR_SIZE`]).
pub trait BlockDevice {
    /// Sector size in bytes (512 for ATA/AHCI).
    fn sector_size(&self) -> usize;
    /// Number of addressable sectors.
    fn sector_count(&self) -> u64;
    /// Read `buf.len() / sector_size` sectors starting at `lba`.
    /// `buf.len()` must be a whole number of sectors.
    fn read_sectors(&mut self, lba: u64, buf: &mut [u8]) -> Result<(), BlockError>;
    /// Write `buf.len() / sector_size` sectors starting at `lba`.
    fn write_sectors(&mut self, lba: u64, buf: &[u8]) -> Result<(), BlockError>;
    /// Push cached writes to the medium.
    fn flush(&mut self) -> Result<(), BlockError>;
}

/// In-memory block device for host tests, backed by a `Vec<u8>`.
///
/// Failure injection: `fail_next_read` / `fail_next_write` make the
/// next operation of that kind return `BlockError::InjectedFailure`
/// once, then clear themselves — useful for testing error paths in
/// the filesystem layer without real hardware.
pub struct MockBlockDevice {
    data: Vec<u8>,
    sectors: u64,
    fail_next_read: bool,
    fail_next_write: bool,
    pub reads: u64,
    pub writes: u64,
    pub flushes: u64,
}

impl MockBlockDevice {
    /// Create a device of `sectors` 512-byte sectors, zero-filled.
    pub fn new(sectors: u64) -> Self {
        Self {
            data: alloc::vec![0u8; sectors as usize * SECTOR_SIZE],
            sectors,
            fail_next_read: false,
            fail_next_write: false,
            reads: 0,
            writes: 0,
            flushes: 0,
        }
    }
    /// Create a device pre-filled with `data` (must be a whole number
    /// of sectors).
    pub fn from_bytes(data: Vec<u8>) -> Self {
        assert_eq!(data.len() % SECTOR_SIZE, 0, "not a whole number of sectors");
        let sectors = (data.len() / SECTOR_SIZE) as u64;
        Self { data, sectors, fail_next_read: false, fail_next_write: false, reads: 0, writes: 0, flushes: 0 }
    }
    pub fn fail_next_read(&mut self) {
        self.fail_next_read = true;
    }
    pub fn fail_next_write(&mut self) {
        self.fail_next_write = true;
    }
    /// Direct access to the backing store (for test assertions).
    pub fn backing(&self) -> &[u8] {
        &self.data
    }
    fn check(&self, lba: u64, buf_len: usize) -> Result<usize, BlockError> {
        if buf_len % SECTOR_SIZE != 0 {
            return Err(BlockError::BadBuffer);
        }
        let count = (buf_len / SECTOR_SIZE) as u64;
        if lba.checked_add(count).map_or(true, |end| end > self.sectors) {
            return Err(BlockError::OutOfRange);
        }
        Ok(count as usize)
    }
}

impl BlockDevice for MockBlockDevice {
    fn sector_size(&self) -> usize {
        SECTOR_SIZE
    }
    fn sector_count(&self) -> u64 {
        self.sectors
    }
    fn read_sectors(&mut self, lba: u64, buf: &mut [u8]) -> Result<(), BlockError> {
        if self.fail_next_read {
            self.fail_next_read = false;
            return Err(BlockError::InjectedFailure);
        }
        let count = self.check(lba, buf.len())?;
        let start = lba as usize * SECTOR_SIZE;
        let end = start + count * SECTOR_SIZE;
        buf.copy_from_slice(&self.data[start..end]);
        self.reads += 1;
        Ok(())
    }
    fn write_sectors(&mut self, lba: u64, buf: &[u8]) -> Result<(), BlockError> {
        if self.fail_next_write {
            self.fail_next_write = false;
            return Err(BlockError::InjectedFailure);
        }
        let count = self.check(lba, buf.len())?;
        let start = lba as usize * SECTOR_SIZE;
        let end = start + count * SECTOR_SIZE;
        self.data[start..end].copy_from_slice(buf);
        self.writes += 1;
        Ok(())
    }
    fn flush(&mut self) -> Result<(), BlockError> {
        self.flushes += 1;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mock_read_write_roundtrip() {
        let mut dev = MockBlockDevice::new(16);
        assert_eq!(dev.sector_size(), 512);
        assert_eq!(dev.sector_count(), 16);
        let mut w = [0u8; 1024];
        for (i, b) in w.iter_mut().enumerate() {
            *b = (i % 251) as u8;
        }
        dev.write_sectors(4, &w).unwrap();
        let mut r = [0u8; 1024];
        dev.read_sectors(4, &mut r).unwrap();
        assert_eq!(r, w);
        assert_eq!(dev.reads, 1);
        assert_eq!(dev.writes, 1);
        dev.flush().unwrap();
        assert_eq!(dev.flushes, 1);
        // Untouched sectors are zero.
        let mut z = [0xFFu8; 512];
        dev.read_sectors(0, &mut z).unwrap();
        assert!(z.iter().all(|b| *b == 0));
    }

    #[test]
    fn mock_out_of_range_and_bad_buffer() {
        let mut dev = MockBlockDevice::new(4);
        let mut buf = [0u8; 512];
        assert_eq!(dev.read_sectors(4, &mut buf), Err(BlockError::OutOfRange));
        assert_eq!(dev.read_sectors(3, &mut buf), Ok(()));
        // Two sectors starting at 3 runs past the end.
        let mut buf2 = [0u8; 1024];
        assert_eq!(dev.read_sectors(3, &mut buf2), Err(BlockError::OutOfRange));
        assert_eq!(dev.write_sectors(0, &[0u8; 100]), Err(BlockError::BadBuffer));
        assert_eq!(dev.read_sectors(0, &mut [0u8; 100]), Err(BlockError::BadBuffer));
    }

    #[test]
    fn mock_failure_injection_fires_once() {
        let mut dev = MockBlockDevice::new(4);
        dev.fail_next_read();
        let mut buf = [0u8; 512];
        assert_eq!(dev.read_sectors(0, &mut buf), Err(BlockError::InjectedFailure));
        // Cleared: the next read works.
        assert_eq!(dev.read_sectors(0, &mut buf), Ok(()));
        dev.fail_next_write();
        assert_eq!(dev.write_sectors(0, &[1u8; 512]), Err(BlockError::InjectedFailure));
        assert_eq!(dev.write_sectors(0, &[1u8; 512]), Ok(()));
    }

    #[test]
    fn mock_from_bytes() {
        let data: Vec<u8> = (0..1024u32).map(|i| (i % 256) as u8).collect();
        let mut dev = MockBlockDevice::from_bytes(data.clone());
        assert_eq!(dev.sector_count(), 2);
        let mut buf = [0u8; 1024];
        dev.read_sectors(0, &mut buf).unwrap();
        assert_eq!(buf.to_vec(), data);
    }
}
