//! CRC16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no xorout).
//! Four bits per step from a 16-entry table built at compile time: 32 bytes
//! of flash. A plain bit-at-a-time loop is no smaller in practice: LLVM 23
//! (rustc 1.99) recognises it and substitutes a 512-byte byte-wise table.

const POLY: u16 = 0x1021;

const NIBBLE: [u16; 16] = {
    let mut t = [0u16; 16];
    let mut i = 0;
    while i < 16 {
        let mut crc = (i as u16) << 12;
        let mut bit = 0;
        while bit < 4 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ POLY
            } else {
                crc << 1
            };
            bit += 1;
        }
        t[i] = crc;
        i += 1;
    }
    t
};

// Not inlined: called from both the RX and TX paths, and an inlined copy
// gets its loop unrolled in each.
#[inline(never)]
pub fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        crc = (crc << 4) ^ NIBBLE[(crc >> 12) as usize];
        crc = (crc << 4) ^ NIBBLE[(crc >> 12) as usize];
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn known_vector() {
        // The classic CRC16/CCITT-FALSE check value.
        assert_eq!(crc16(b"123456789"), 0x29B1);
    }

    /// Matches the bit-at-a-time definition on every single byte and on a
    /// longer run.
    #[test]
    fn matches_bitwise() {
        fn bitwise(data: &[u8]) -> u16 {
            let mut crc: u16 = 0xFFFF;
            for &byte in data {
                crc ^= (byte as u16) << 8;
                for _ in 0..8 {
                    crc = if crc & 0x8000 != 0 {
                        (crc << 1) ^ POLY
                    } else {
                        crc << 1
                    };
                }
            }
            crc
        }
        for b in 0..=255u8 {
            assert_eq!(crc16(&[b]), bitwise(&[b]));
        }
        let run: [u8; 300] = core::array::from_fn(|i| (i * 7 + 3) as u8);
        assert_eq!(crc16(&run), bitwise(&run));
    }

    #[test]
    fn empty_is_init() {
        assert_eq!(crc16(&[]), 0xFFFF);
    }
}
