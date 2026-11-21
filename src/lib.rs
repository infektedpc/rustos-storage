//! `rustos-storage`: block storage for RustOS.
//!
//! Layers, bottom to top:
//!
//! ```text
//! rustfs (rustos-fs, 4096-byte blocks)
//!       │  read_block / write_block (8 × 512-byte sectors)
//! BlockDevice trait ─────────────────────────────┐
//!       │                                        │
//!   AtaDisk (PIO)                          AhciPort (DMA)
//!   ports 0x1F0/0x170                    HBA MMIO (PCI BAR5)
//!       │                                        │
//!   IDE controller                     AHCI controller (PCI class 01/06)
//! ```
//!
//! The crate is `no_std` outside tests (`core` + `alloc` only, zero
//! external crates) so it can link into `rustos-kernel`. Every driver is
//! generic over a hardware trait (`PortIo` / `Mmio`) and ships with an
//! in-memory mock, so all of it is unit-testable on the host.
//!
//! # Modules
//!
//! * [`block`] — the [`block::BlockDevice`] trait and [`block::MockBlockDevice`]
//! * [`ata`] — ATA/IDE PIO driver (LBA28 + LBA48, IDENTIFY)
//! * [`ahci`] — AHCI SATA: HBA layout, command headers/tables, FIS
//! * [`partition`] — MBR + GPT parsing, CRC-32
//! * [`ports`] — port I/O abstraction + mock ports (same contract as
//!   `rustos-drivers::ports`, self-contained, but with a wider range
//!   covering the PCI ports 0xCF8/0xCFC and a data-port FIFO so ATA PIO
//!   transfers can be tested end to end)

#![cfg_attr(not(test), no_std)]

extern crate alloc;

pub mod ahci;
pub mod ata;
pub mod block;
pub mod partition;
pub mod ports;

pub use ata::AtaDisk;
pub use block::{BlockDevice, BlockError, MockBlockDevice};

/// Sector size every driver in this crate uses: 512 bytes.
pub const SECTOR_SIZE: usize = 512;
/// rustfs block size (see `rustos-fs`): one block = 8 sectors.
pub const RUSTFS_BLOCK_SIZE: usize = 4096;
/// Sectors per rustfs block.
pub const SECTORS_PER_RUSTFS_BLOCK: u64 = (RUSTFS_BLOCK_SIZE / SECTOR_SIZE) as u64;

/// PCI class/subclass/programming-interface of an AHCI controller.
/// The kernel finds it with `rustos-nic` PCI enumeration and passes
/// BAR5 (the HBA MMIO base) to [`ahci::AhciHba`].
pub const AHCI_PCI_CLASS: u8 = 0x01;
pub const AHCI_PCI_SUBCLASS: u8 = 0x06;
pub const AHCI_PCI_PROG_IF: u8 = 0x01;

/// Read one rustfs block (4096 bytes) from a block device.
/// `block_no` is the rustfs block number, not a sector number.
pub fn read_rustfs_block<D: BlockDevice>(
    dev: &mut D,
    block_no: u64,
    out: &mut [u8; RUSTFS_BLOCK_SIZE],
) -> Result<(), BlockError> {
    dev.read_sectors(block_no * SECTORS_PER_RUSTFS_BLOCK, out)
}

/// Write one rustfs block (4096 bytes) to a block device.
pub fn write_rustfs_block<D: BlockDevice>(
    dev: &mut D,
    block_no: u64,
    data: &[u8; RUSTFS_BLOCK_SIZE],
) -> Result<(), BlockError> {
    dev.write_sectors(block_no * SECTORS_PER_RUSTFS_BLOCK, data)
}
