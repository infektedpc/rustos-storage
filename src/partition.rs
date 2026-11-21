//! MBR and GPT partition-table parsing.
//!
//! Both parsers work on raw sector bytes, so they run on the host
//! against a fake image and in the kernel against a real
//! [`crate::block::BlockDevice`]. Nothing here touches hardware.
//!
//! # MBR
//!
//! Sector 0, signature `0x55AA` at offset 510. Four 16-byte entries
//! at offset 446: boot flag (0x80), CHS start (3 bytes, ignored),
//! type (1 byte), CHS end (3 bytes, ignored), LBA start (u32 LE),
//! sector count (u32 LE).
//!
//! # GPT
//!
//! A protective MBR (one entry of type 0xEE spanning the disk), then
//! the GPT header at LBA 1 ("EFI PART"), then the entry array (128
//! entries × 128 bytes by default, at LBA 2). Each entry: type GUID
//! (16 bytes), unique GUID (16 bytes), first LBA (u64), last LBA
//! (u64), attributes (u64), name (72 bytes UTF-16LE). The header
//! carries a CRC-32 (IEEE, the same polynomial as Ethernet/ZIP) of
//! itself (with its own CRC field zeroed) and of the entry array.

use alloc::{string::String, vec::Vec};

/// CRC-32/IEEE of `data` (polynomial 0xEDB88320, init 0xFFFF_FFFF,
/// final XOR 0xFFFF_FFFF). Table-free, bit-by-bit — fast enough for
/// a 512-byte header and a 16 KiB entry array at boot.
pub fn crc32(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for &b in data {
        crc ^= b as u32;
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

/// A partition-table entry, normalised across MBR and GPT.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Partition {
    /// Index in the table (0..3 for MBR, 0..127 for GPT).
    pub index: usize,
    /// What kind of partition this is.
    pub kind: PartitionKind,
    /// First sector (LBA).
    pub first_lba: u64,
    /// Last sector (LBA, inclusive).
    pub last_lba: u64,
    /// GPT name, if any (MBR entries have no name).
    pub name: [u8; 36],
    /// Name length in UTF-16 code units (0 for MBR).
    pub name_len: usize,
    /// Bootable flag (MBR boot byte 0x80; GPT has no such flag).
    pub bootable: bool,
}

impl Partition {
    /// Number of sectors in the partition.
    pub fn sector_count(&self) -> u64 {
        self.last_lba.saturating_sub(self.first_lba) + 1
    }
    pub fn is_empty(&self) -> bool {
        matches!(self.kind, PartitionKind::Empty)
    }
    /// Name as a lossy UTF-16 string (GPT only; empty for MBR).
    /// `name` stores one byte per UTF-16 code unit (ASCII names);
    /// non-ASCII names lose their high byte, which is documented.
    pub fn name_str(&self) -> String {
        let units: Vec<u16> = (0..self.name_len).map(|i| self.name[i] as u16).collect();
        String::from_utf16_lossy(&units)
    }
}

/// What a partition holds, by MBR type byte or GPT type GUID.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PartitionKind {
    Empty,
    /// MBR type 0xEE: protective MBR (the disk is GPT).
    ProtectiveMbr,
    /// EFI System Partition.
    EfiSystem,
    /// Linux filesystem partition (MBR 0x83 / GPT Linux GUID).
    LinuxFilesystem,
    /// rustfs partition (MBR type 0xDA / GPT RustOS GUID).
    RustFs,
    /// Microsoft basic data (MBR 0x07 / GPT basic-data GUID).
    MicrosoftBasic,
    /// Something else — raw type byte / GUID kept in `other`.
    Other([u8; 16]),
}

/// MBR type bytes.
pub const MBR_TYPE_EMPTY: u8 = 0x00;
pub const MBR_TYPE_EFI_SYSTEM: u8 = 0xEF;
pub const MBR_TYPE_LINUX: u8 = 0x83;
pub const MBR_TYPE_RUSTFS: u8 = 0xDA;
pub const MBR_TYPE_MICROSOFT_BASIC: u8 = 0x07;
pub const MBR_TYPE_PROTECTIVE: u8 = 0xEE;

/// GPT type GUIDs (stored little-endian per the GPT spec's mixed
/// endianness: first three fields little-endian, rest as written).
/// Bytes here are the on-disk byte order.
pub const GUID_EFI_SYSTEM: [u8; 16] = [
    0x28, 0x73, 0x2A, 0xC1, 0x1F, 0xF8, 0xD2, 0x11, 0xBA, 0x4B, 0x00, 0xA0, 0xC9, 0x3E, 0xC9, 0x3B,
];
pub const GUID_LINUX_FILESYSTEM: [u8; 16] = [
    0xAF, 0x3D, 0xC6, 0x0F, 0x83, 0x84, 0x72, 0x47, 0x8E, 0x79, 0x3D, 0x69, 0xD8, 0x47, 0x7D, 0xE4,
];
pub const GUID_MICROSOFT_BASIC: [u8; 16] = [
    0xA2, 0xA0, 0xD0, 0xEB, 0xE5, 0xB9, 0x33, 0x44, 0x87, 0xC0, 0x68, 0xB6, 0xB7, 0x26, 0x99, 0xC7,
];
/// RustOS rustfs partition GUID: {52 55 53 54-46 53-31 00-00-00-00-00-00-00-00-00}
/// ("RUST-FS-1" in the first fields). Chosen by this project; not a
/// registered GUID.
pub const GUID_RUSTFS: [u8; 16] = [
    0x52, 0x55, 0x53, 0x54, 0x46, 0x53, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
];

fn kind_from_mbr_type(t: u8) -> PartitionKind {
    match t {
        MBR_TYPE_EMPTY => PartitionKind::Empty,
        MBR_TYPE_PROTECTIVE => PartitionKind::ProtectiveMbr,
        MBR_TYPE_EFI_SYSTEM => PartitionKind::EfiSystem,
        MBR_TYPE_LINUX => PartitionKind::LinuxFilesystem,
        MBR_TYPE_RUSTFS => PartitionKind::RustFs,
        MBR_TYPE_MICROSOFT_BASIC => PartitionKind::MicrosoftBasic,
        _ => PartitionKind::Other({
            let mut g = [0u8; 16];
            g[0] = t;
            g
        }),
    }
}

fn kind_from_guid(g: &[u8; 16]) -> PartitionKind {
    if *g == [0u8; 16] {
        PartitionKind::Empty
    } else if *g == GUID_EFI_SYSTEM {
        PartitionKind::EfiSystem
    } else if *g == GUID_LINUX_FILESYSTEM {
        PartitionKind::LinuxFilesystem
    } else if *g == GUID_RUSTFS {
        PartitionKind::RustFs
    } else if *g == GUID_MICROSOFT_BASIC {
        PartitionKind::MicrosoftBasic
    } else {
        PartitionKind::Other(*g)
    }
}

/// Why a partition table could not be parsed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PartitionError {
    /// Sector 0 is shorter than 512 bytes or has no 0x55AA signature.
    BadMbrSignature,
    /// The GPT header signature is not "EFI PART".
    BadGptSignature,
    /// The GPT header CRC does not match.
    BadGptHeaderCrc,
    /// The GPT entry-array CRC does not match.
    BadGptEntriesCrc,
    /// Not enough data was supplied for the table being parsed.
    Truncated,
}

/// Parse the MBR in sector 0 (`sector` must be at least 512 bytes).
/// Returns all four entries, including empty ones.
pub fn parse_mbr(sector: &[u8]) -> Result<[Partition; 4], PartitionError> {
    if sector.len() < 512 || sector[510] != 0x55 || sector[511] != 0xAA {
        return Err(PartitionError::BadMbrSignature);
    }
    let mut out = [
        Partition {
            index: 0,
            kind: PartitionKind::Empty,
            first_lba: 0,
            last_lba: 0,
            name: [0; 36],
            name_len: 0,
            bootable: false,
        },
        Partition {
            index: 1,
            kind: PartitionKind::Empty,
            first_lba: 0,
            last_lba: 0,
            name: [0; 36],
            name_len: 0,
            bootable: false,
        },
        Partition {
            index: 2,
            kind: PartitionKind::Empty,
            first_lba: 0,
            last_lba: 0,
            name: [0; 36],
            name_len: 0,
            bootable: false,
        },
        Partition {
            index: 3,
            kind: PartitionKind::Empty,
            first_lba: 0,
            last_lba: 0,
            name: [0; 36],
            name_len: 0,
            bootable: false,
        },
    ];
    for i in 0..4 {
        let e = &sector[446 + i * 16..446 + (i + 1) * 16];
        let ty = e[4];
        let first = u32::from_le_bytes(e[8..12].try_into().unwrap()) as u64;
        let count = u32::from_le_bytes(e[12..16].try_into().unwrap()) as u64;
        out[i] = Partition {
            index: i,
            kind: kind_from_mbr_type(ty),
            first_lba: first,
            last_lba: first.saturating_add(count).saturating_sub(1),
            name: [0; 36],
            name_len: 0,
            bootable: e[0] == 0x80,
        };
    }
    Ok(out)
}

/// Is sector 0 a protective MBR (single 0xEE entry)? If so the disk
/// is GPT and [`parse_gpt`] should be used.
pub fn is_protective_mbr(sector: &[u8]) -> bool {
    matches!(parse_mbr(sector), Ok(e) if e[0].kind == PartitionKind::ProtectiveMbr)
}

/// Parse a GPT disk. `image` is the raw disk bytes starting at LBA 0
/// (must cover at least the header at LBA 1 and the entry array).
/// Returns the non-empty partitions.
pub fn parse_gpt(image: &[u8]) -> Result<Vec<Partition>, PartitionError> {
    // Header is at LBA 1 = byte offset 512, 92 bytes minimum.
    if image.len() < 512 + 92 {
        return Err(PartitionError::Truncated);
    }
    let header = &image[512..];
    if &header[0..8] != b"EFI PART" {
        return Err(PartitionError::BadGptSignature);
    }
    let header_size = u32::from_le_bytes(header[12..16].try_into().unwrap()) as usize;
    if header_size < 92 || image.len() < 512 + header_size {
        return Err(PartitionError::Truncated);
    }
    let stored_crc = u32::from_le_bytes(header[16..20].try_into().unwrap());
    let mut header_for_crc = image[512..512 + header_size].to_vec();
    header_for_crc[16..20].copy_from_slice(&[0, 0, 0, 0]);
    if crc32(&header_for_crc) != stored_crc {
        return Err(PartitionError::BadGptHeaderCrc);
    }
    let entries_lba = u64::from_le_bytes(header[72..80].try_into().unwrap());
    let entry_count = u32::from_le_bytes(header[80..84].try_into().unwrap()) as usize;
    let entry_size = u32::from_le_bytes(header[84..88].try_into().unwrap()) as usize;
    let entries_crc = u32::from_le_bytes(header[88..92].try_into().unwrap());
    let entries_off = entries_lba as usize * 512;
    let entries_len = entry_count * entry_size;
    if entry_size < 128 || image.len() < entries_off + entries_len {
        return Err(PartitionError::Truncated);
    }
    if crc32(&image[entries_off..entries_off + entries_len]) != entries_crc {
        return Err(PartitionError::BadGptEntriesCrc);
    }
    let mut out = Vec::new();
    for i in 0..entry_count {
        let e = &image[entries_off + i * entry_size..entries_off + (i + 1) * entry_size];
        let mut type_guid = [0u8; 16];
        type_guid.copy_from_slice(&e[0..16]);
        let kind = kind_from_guid(&type_guid);
        if kind == PartitionKind::Empty {
            continue;
        }
        let first = u64::from_le_bytes(e[32..40].try_into().unwrap());
        let last = u64::from_le_bytes(e[40..48].try_into().unwrap());
        // Name: 72 bytes UTF-16LE starting at offset 56, up to 36 units.
        let mut name = [0u8; 36];
        let mut name_len = 0usize;
        for u in 0..36 {
            let lo = e[56 + u * 2];
            let hi = e[56 + u * 2 + 1];
            if lo == 0 && hi == 0 {
                break;
            }
            // Store only the low byte (names here are ASCII).
            name[u] = lo;
            name_len = u + 1;
        }
        out.push(Partition {
            index: i,
            kind,
            first_lba: first,
            last_lba: last,
            name,
            name_len,
            bootable: false,
        });
    }
    Ok(out)
}

/// Find the first partition of a given kind, if any.
pub fn find_partition<'a>(parts: &'a [Partition], kind: PartitionKind) -> Option<&'a Partition> {
    parts.iter().find(|p| p.kind == kind)
}

/// Convenience: parse sector 0 as MBR, or the whole image as GPT
/// when it is a protective MBR. Returns the non-empty partitions.
pub fn parse_partitions(image: &[u8]) -> Result<Vec<Partition>, PartitionError> {
    if image.len() < 512 {
        return Err(PartitionError::Truncated);
    }
    if is_protective_mbr(&image[..512]) {
        parse_gpt(image)
    } else {
        let mbr = parse_mbr(&image[..512])?;
        Ok(mbr.into_iter().filter(|p| !p.is_empty()).collect())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;

    fn mbr_sector(entries: &[(u8, u8, u32, u32)]) -> [u8; 512] {
        // (boot, type, first_lba, count)
        let mut s = [0u8; 512];
        s[510] = 0x55;
        s[511] = 0xAA;
        for (i, (boot, ty, first, count)) in entries.iter().enumerate() {
            let e = &mut s[446 + i * 16..446 + (i + 1) * 16];
            e[0] = *boot;
            e[4] = *ty;
            e[8..12].copy_from_slice(&first.to_le_bytes());
            e[12..16].copy_from_slice(&count.to_le_bytes());
        }
        s
    }

    /// Build a fake GPT image: protective MBR + header + 128 entries.
    /// `entries`: (type GUID, first LBA, last LBA, name).
    fn gpt_image(entries: &[([u8; 16], u64, u64, &str)]) -> Vec<u8> {
        let entry_count = 128usize;
        let entry_size = 128usize;
        let entries_bytes = entry_count * entry_size;
        // Header (92 bytes) + entries; image = LBA0 + LBA1 + entries.
        let mut image = vec![0u8; 512 + 512 + entries_bytes];
        // Protective MBR: one 0xEE entry spanning the disk.
        image[510] = 0x55;
        image[511] = 0xAA;
        image[446] = 0x00;
        image[450] = MBR_TYPE_PROTECTIVE;
        image[454..458].copy_from_slice(&1u32.to_le_bytes());
        image[458..462].copy_from_slice(&0xFFFF_FFFFu32.to_le_bytes());
        // Entries at LBA 2.
        let entries_off = 1024;
        for (i, (guid, first, last, name)) in entries.iter().enumerate() {
            let e = &mut image[entries_off + i * entry_size..entries_off + (i + 1) * entry_size];
            e[0..16].copy_from_slice(guid);
            e[16..32].copy_from_slice(&[0x11u8; 16]); // unique GUID
            e[32..40].copy_from_slice(&first.to_le_bytes());
            e[40..48].copy_from_slice(&last.to_le_bytes());
            for (j, c) in name.encode_utf16().enumerate().take(36) {
                e[56 + j * 2..58 + j * 2].copy_from_slice(&c.to_le_bytes());
            }
        }
        let entries_crc = crc32(&image[entries_off..entries_off + entries_bytes]);
        // Header at LBA 1.
        let h = &mut image[512..512 + 92];
        h[0..8].copy_from_slice(b"EFI PART");
        h[8..12].copy_from_slice(&0x0001_0000u32.to_le_bytes()); // revision 1.0
        h[12..16].copy_from_slice(&92u32.to_le_bytes());
        h[16..20].copy_from_slice(&0u32.to_le_bytes()); // CRC, filled below
        h[24..32].copy_from_slice(&1u64.to_le_bytes()); // current LBA
        h[32..40].copy_from_slice(&100u64.to_le_bytes()); // backup LBA
        h[40..48].copy_from_slice(&34u64.to_le_bytes()); // first usable
        h[48..56].copy_from_slice(&99u64.to_le_bytes()); // last usable
        h[56..72].copy_from_slice(&[0x22u8; 16]); // disk GUID
        h[72..80].copy_from_slice(&2u64.to_le_bytes()); // entries LBA
        h[80..84].copy_from_slice(&(entry_count as u32).to_le_bytes());
        h[84..88].copy_from_slice(&(entry_size as u32).to_le_bytes());
        h[88..92].copy_from_slice(&entries_crc.to_le_bytes());
        let header_crc = crc32(&image[512..512 + 92]);
        image[512 + 16..512 + 20].copy_from_slice(&header_crc.to_le_bytes());
        image
    }

    #[test]
    fn crc32_known_vector() {
        // "123456789" -> 0xCBF43926 (the standard check value).
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
        assert_eq!(crc32(b""), 0);
    }

    #[test]
    fn mbr_parse() {
        let s = mbr_sector(&[
            (0x80, MBR_TYPE_LINUX, 2048, 100_000),
            (0x00, MBR_TYPE_EFI_SYSTEM, 102_048, 2048),
            (0x00, MBR_TYPE_EMPTY, 0, 0),
            (0x00, MBR_TYPE_RUSTFS, 200_000, 50_000),
        ]);
        let parts = parse_mbr(&s).unwrap();
        assert_eq!(parts[0].kind, PartitionKind::LinuxFilesystem);
        assert!(parts[0].bootable);
        assert_eq!(parts[0].first_lba, 2048);
        assert_eq!(parts[0].last_lba, 2048 + 100_000 - 1);
        assert_eq!(parts[0].sector_count(), 100_000);
        assert_eq!(parts[1].kind, PartitionKind::EfiSystem);
        assert_eq!(parts[1].first_lba, 102_048);
        assert!(parts[2].is_empty());
        assert_eq!(parts[3].kind, PartitionKind::RustFs);
        assert_eq!(find_partition(&parts, PartitionKind::RustFs).unwrap().first_lba, 200_000);
    }

    #[test]
    fn mbr_bad_signature() {
        let mut s = mbr_sector(&[]);
        s[511] = 0x00;
        assert_eq!(parse_mbr(&s), Err(PartitionError::BadMbrSignature));
        assert_eq!(parse_mbr(&[0u8; 100]), Err(PartitionError::BadMbrSignature));
    }

    #[test]
    fn gpt_parse() {
        let image = gpt_image(&[
            (GUID_EFI_SYSTEM, 2048, 4095, "EFI System"),
            (GUID_RUSTFS, 4096, 100_000, "rustfs"),
            (GUID_LINUX_FILESYSTEM, 100_001, 200_000, "root"),
        ]);
        assert!(is_protective_mbr(&image[..512]));
        let parts = parse_gpt(&image).unwrap();
        assert_eq!(parts.len(), 3);
        assert_eq!(parts[0].kind, PartitionKind::EfiSystem);
        assert_eq!(parts[0].name_str(), "EFI System");
        assert_eq!(parts[1].kind, PartitionKind::RustFs);
        assert_eq!(parts[1].first_lba, 4096);
        assert_eq!(parts[1].last_lba, 100_000);
        assert_eq!(parts[1].name_str(), "rustfs");
        assert_eq!(parts[2].kind, PartitionKind::LinuxFilesystem);
        assert_eq!(find_partition(&parts, PartitionKind::RustFs).unwrap().index, 1);
    }

    #[test]
    fn gpt_bad_header_crc() {
        let mut image = gpt_image(&[(GUID_EFI_SYSTEM, 2048, 4095, "EFI")]);
        image[512 + 40] ^= 0xFF; // corrupt first-usable LBA
        assert_eq!(parse_gpt(&image), Err(PartitionError::BadGptHeaderCrc));
    }

    #[test]
    fn gpt_bad_entries_crc() {
        let mut image = gpt_image(&[(GUID_EFI_SYSTEM, 2048, 4095, "EFI")]);
        image[1024 + 40] ^= 0xFF; // corrupt an entry's last LBA
        assert_eq!(parse_gpt(&image), Err(PartitionError::BadGptEntriesCrc));
    }

    #[test]
    fn gpt_bad_signature() {
        let mut image = gpt_image(&[]);
        image[512] = b'X';
        assert_eq!(parse_gpt(&image), Err(PartitionError::BadGptSignature));
    }

    #[test]
    fn parse_partitions_dispatches() {
        // Plain MBR disk.
        let s = mbr_sector(&[(0x00, MBR_TYPE_LINUX, 2048, 1000)]);
        let mut image = s.to_vec();
        image.extend_from_slice(&[0u8; 512]);
        let parts = parse_partitions(&image).unwrap();
        assert_eq!(parts.len(), 1);
        assert_eq!(parts[0].kind, PartitionKind::LinuxFilesystem);
        // GPT disk.
        let gpt = gpt_image(&[(GUID_RUSTFS, 4096, 8192, "rustfs")]);
        let parts = parse_partitions(&gpt).unwrap();
        assert_eq!(parts.len(), 1);
        assert_eq!(parts[0].kind, PartitionKind::RustFs);
        // Too short.
        assert_eq!(parse_partitions(&[0u8; 100]), Err(PartitionError::Truncated));
    }
}
