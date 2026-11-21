//! Port I/O abstraction + mock ports for host testing.
//!
//! Same contract as `rustos-drivers::ports` (and `rustos-nic::ports`),
//! self-contained so this crate has no sibling dependency. Two
//! differences from the drivers version:
//!
//! * The register file covers ports 0x0000..=0x0FFF, so the PCI ports
//!   0xCF8/0xCFC are in range as well as the ATA ports.
//! * The mock has a **data-port FIFO**: ATA PIO transfers read/write
//!   16-bit words on the data port (0x1F0 / 0x170). `feed_data_words`
//!   queues words for the driver to read; words the driver writes are
//!   captured in `data_written`. Without this, a flat register file
//!   would return the same word 256 times and PIO could not be tested.

use alloc::vec::Vec;

/// Byte-wide x86 I/O-port access.
pub trait PortIo {
    fn read_u8(&mut self, port: u16) -> u8;
    fn write_u8(&mut self, port: u16, value: u8);
    fn read_u16(&mut self, port: u16) -> u16 {
        let lo = self.read_u8(port) as u16;
        let hi = self.read_u8(port + 1) as u16;
        (hi << 8) | lo
    }
    fn write_u16(&mut self, port: u16, value: u16) {
        self.write_u8(port, value as u8);
        self.write_u8(port + 1, (value >> 8) as u8);
    }
    fn read_u32(&mut self, port: u16) -> u32 {
        let lo = self.read_u16(port) as u32;
        let hi = self.read_u16(port + 2) as u32;
        (hi << 16) | lo
    }
    fn write_u32(&mut self, port: u16, value: u32) {
        self.write_u16(port, value as u16);
        self.write_u16(port + 2, (value >> 16) as u16);
    }
}

/// In-memory fake ports for host tests. Covers 0x0000..=0x0FFF.
/// Every write is appended to a log so tests can assert exactly what
/// a driver emitted.
pub struct MockPorts {
    regs: [u8; 4096],
    write_log: Vec<(u16, u8)>,
    /// Words queued for 16-bit reads on a data port.
    data_in: Vec<u16>,
    data_in_pos: usize,
    /// Words captured from 16-bit writes on a data port.
    data_written: Vec<u16>,
    /// Ports treated as data ports (ATA data registers). Defaults to
    /// the ATA primary/secondary data ports.
    data_ports: [u16; 2],
}

impl MockPorts {
    pub fn new() -> Self {
        Self {
            regs: [0; 4096],
            write_log: Vec::new(),
            data_in: Vec::new(),
            data_in_pos: 0,
            data_written: Vec::new(),
            data_ports: [0x1F0, 0x170],
        }
    }
    /// Preset a port's read value.
    pub fn set(&mut self, port: u16, value: u8) {
        if (port as usize) < self.regs.len() {
            self.regs[port as usize] = value;
        }
    }
    pub fn get(&self, port: u16) -> u8 {
        if (port as usize) < self.regs.len() {
            self.regs[port as usize]
        } else {
            0
        }
    }
    /// Preset a 16-bit port value (little-endian).
    pub fn set_u16(&mut self, port: u16, value: u16) {
        self.set(port, value as u8);
        self.set(port + 1, (value >> 8) as u8);
    }
    /// Preset a 32-bit port value (little-endian).
    pub fn set_u32(&mut self, port: u16, value: u32) {
        self.set_u16(port, value as u16);
        self.set_u16(port + 2, (value >> 16) as u16);
    }
    /// Queue words for the driver to read from a data port, in order.
    pub fn feed_data_words(&mut self, words: &[u16]) {
        self.data_in.extend_from_slice(words);
    }
    /// Words the driver wrote to a data port, in order.
    pub fn data_written(&self) -> &[u16] {
        &self.data_written
    }
    /// Words still queued for reading.
    pub fn data_remaining(&self) -> usize {
        self.data_in.len() - self.data_in_pos
    }
    pub fn write_log_len(&self) -> usize {
        self.write_log.len()
    }
    pub fn write_log_entry(&self, i: usize) -> Option<(u16, u8)> {
        self.write_log.get(i).copied()
    }
    /// Bytes written to `port`, in order.
    pub fn bytes_written_to(&self, port: u16, out: &mut [u8]) -> usize {
        let mut n = 0;
        for (p, v) in &self.write_log {
            if *p == port && n < out.len() {
                out[n] = *v;
                n += 1;
            }
        }
        n
    }
    /// Did the driver write `value` to `port` at any point?
    pub fn wrote(&self, port: u16, value: u8) -> bool {
        self.write_log.iter().any(|(p, v)| *p == port && *v == value)
    }
    fn is_data_port(&self, port: u16) -> bool {
        self.data_ports.contains(&port)
    }
}

impl Default for MockPorts {
    fn default() -> Self {
        Self::new()
    }
}

impl PortIo for MockPorts {
    fn read_u8(&mut self, port: u16) -> u8 {
        self.get(port)
    }
    fn write_u8(&mut self, port: u16, value: u8) {
        // The ATA status/command port (0x1F7 / 0x177) is read as
        // status but written as command: a command write must not
        // clobber the status value a test preset with `set()`.
        if port != 0x1F7 && port != 0x177 && (port as usize) < self.regs.len() {
            self.regs[port as usize] = value;
        }
        self.write_log.push((port, value));
    }
    fn read_u16(&mut self, port: u16) -> u16 {
        if self.is_data_port(port) && self.data_in_pos < self.data_in.len() {
            let w = self.data_in[self.data_in_pos];
            self.data_in_pos += 1;
            w
        } else {
            let lo = self.read_u8(port) as u16;
            let hi = self.read_u8(port + 1) as u16;
            (hi << 8) | lo
        }
    }
    fn write_u16(&mut self, port: u16, value: u16) {
        if self.is_data_port(port) {
            self.data_written.push(value);
        }
        self.write_u8(port, value as u8);
        self.write_u8(port + 1, (value >> 8) as u8);
    }
}

/// Real x86_64 port I/O via `in`/`out`. Only on x86_64.
#[cfg(target_arch = "x86_64")]
pub struct RealPorts;

#[cfg(target_arch = "x86_64")]
impl PortIo for RealPorts {
    fn read_u8(&mut self, port: u16) -> u8 {
        let value: u8;
        unsafe {
            core::arch::asm!("in al, dx", in("dx") port, out("al") value, options(nomem, nostack, preserves_flags));
        }
        value
    }
    fn write_u8(&mut self, port: u16, value: u8) {
        unsafe {
            core::arch::asm!("out dx, al", in("dx") port, in("al") value, options(nomem, nostack, preserves_flags));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn mock_ports_roundtrip_and_log() {
        let mut m = MockPorts::new();
        m.write_u8(0x3f8, b'A');
        assert_eq!(m.read_u8(0x3f8), b'A');
        assert!(m.wrote(0x3f8, b'A'));
        assert!(!m.wrote(0x3f8, b'B'));
        let mut out = [0u8; 4];
        assert_eq!(m.bytes_written_to(0x3f8, &mut out), 1);
        assert_eq!(out[0], b'A');
        m.set(0x60, 0x1e);
        assert_eq!(m.read_u8(0x60), 0x1e);
        // PCI ports are in range.
        m.set_u32(0xCFC, 0x8086_100E);
        assert_eq!(m.read_u32(0xCFC), 0x8086_100E);
    }

    #[test]
    fn status_port_write_does_not_clobber_status() {
        // ATA status (read) and command (write) share port 0x1F7.
        let mut m = MockPorts::new();
        m.set(0x1F7, 0x48); // DRDY | DRQ
        m.write_u8(0x1F7, 0x20); // READ SECTORS command
        assert_eq!(m.read_u8(0x1F7), 0x48);
        assert!(m.wrote(0x1F7, 0x20));
    }

    #[test]
    fn data_port_fifo() {
        let mut m = MockPorts::new();
        m.feed_data_words(&[0x1234, 0xABCD]);
        assert_eq!(m.read_u16(0x1F0), 0x1234);
        assert_eq!(m.read_u16(0x1F0), 0xABCD);
        assert_eq!(m.data_remaining(), 0);
        m.write_u16(0x1F0, 0x55AA);
        assert_eq!(m.data_written(), &[0x55AA]);
        // Non-data ports still use the register file.
        assert_eq!(m.read_u16(0x40), 0);
    }
}
