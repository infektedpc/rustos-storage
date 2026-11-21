# rustos-storage

Block storage for RustOS: the block-device trait, an ATA/IDE PIO
driver, an AHCI SATA driver, and MBR/GPT partition parsing.
`no_std` outside tests (`core` + `alloc` only, zero external
crates).

## Layers

```text
rustfs (rustos-fs, 4096-byte blocks)
      │  read_rustfs_block / write_rustfs_block (8 × 512-byte sectors)
BlockDevice trait
      │
  AtaDisk (PIO)                AhciPort (DMA)
  ports 0x1F0/0x170            HBA MMIO (PCI BAR5)
      │                            │
  IDE controller             AHCI controller (PCI 01/06/01)
```

## What is here

| Module | Contents |
|---|---|
| `block` | `BlockDevice` trait (read/write/flush sectors), `MockBlockDevice` with failure injection |
| `ata` | ATA/IDE PIO: IDENTIFY, READ/WRITE SECTORS (LBA28 + LBA48), FLUSH CACHE; one command per sector |
| `ahci` | HBA layout, command headers/tables, Register–Host-to-Device FIS, PRDT entries, port start/stop/issue |
| `partition` | MBR parsing, GPT parsing (header + entry-array CRC-32), partition lookup by kind |
| `ports` | `PortIo` trait + `MockPorts` (same contract as `rustos-drivers::ports`, plus a data-port FIFO so ATA PIO transfers can be tested) |

## How a read flows

```text
rustfs block 7
  -> read_rustfs_block(dev, 7)          // sectors 56..63
  -> BlockDevice::read_sectors(56, &mut [u8; 4096])
  -> AtaDisk: program LBA 56, issue READ SECTORS, poll DRQ,
     read 256 words per sector from the data port
  -> AhciPort: build command header + FIS + PRDT, set CI bit,
     hardware DMAs the data into the buffer
```

## Wiring it into the kernel

```rust
// ATA: primary channel, master drive.
let mut disk = AtaDisk::new(RealPorts, Channel::Primary, Drive::Master);
let info = disk.identify()?;           // fills sectors + LBA48 flag
disk.read_sectors(0, &mut sector)?;

// AHCI: find the controller with rustos-nic PCI enumeration
// (class 0x01, subclass 0x06, prog-if 0x01), map BAR5, then:
let mut hba = AhciHba::new(mmio);
hba.enable();
let ports = hba.active_ports();        // implemented + device present
hba.set_port_buffers(0, clb_phys, fb_phys);
hba.start_port(0);
// Build a command header + table (see build_command_table),
// write the header into the command list, then:
hba.issue_command(0, 0);

// Partitions: read sector 0, then:
let parts = parse_partitions(&image)?;
let rustfs = find_partition(&parts, PartitionKind::RustFs).unwrap();
```

DMA buffers (command list, FIS area, data buffers) must be real,
physically-contiguous memory — allocate them from
`rustos-memory`'s frame allocator and map them before handing
their physical addresses to AHCI.

## Tests

```sh
cargo test
```

All drivers are generic over a hardware trait (`PortIo` / `Mmio`)
with in-memory mocks, so the tests run on the host with no
hardware: ATA IDENTIFY parsing and PIO transfers through the
mock data FIFO, AHCI command-header/table layout and register
pokes, MBR/GPT parsing against a fake GPT image built in-memory
(including CRC-32 checks).

## Limitations

- **ATA is PIO only** — no DMA/UDMA. One command per sector.
  For DMA use AHCI.
- **AHCI needs real DMA buffers** from `rustos-memory`; this crate
  builds the structures and pokes registers but never performs DMA
  itself. No NCQ, no ATAPI, no port-multiplier support.
- **No SMART, no TRIM, no power management.**
- **rustfs blocks are 4096 bytes** (8 sectors); a block device with
  a different sector size is not supported.
- **Not live-tested** — QEMU is not installed on this VM and there
  is no real disk hardware here, so no sector has ever been read
  from a real drive. Everything is verified against mocks.
