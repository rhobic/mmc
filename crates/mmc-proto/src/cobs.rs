//! Consistent Overhead Byte Stuffing: removes `0x00` from a frame so it can
//! serve as an unambiguous delimiter on the wire.

/// Worst-case encoded length for `n` input bytes (excluding any delimiter).
pub const fn max_encoded_len(n: usize) -> usize {
    n + n / 254 + 1
}

/// Encode `src` into `dst` (no delimiter appended). Returns the encoded
/// length, or `None` if `dst` is too small — size it with [`max_encoded_len`].
pub fn encode(src: &[u8], dst: &mut [u8]) -> Option<usize> {
    if dst.len() < max_encoded_len(src.len()) {
        return None;
    }
    let mut code_at = 0; // where the current block's code byte lives
    let mut out = 1;
    let mut code: u8 = 1;
    for &b in src {
        if b == 0 {
            dst[code_at] = code;
            code_at = out;
            out += 1;
            code = 1;
        } else {
            dst[out] = b;
            out += 1;
            code += 1;
            if code == 0xFF {
                dst[code_at] = code;
                code_at = out;
                out += 1;
                code = 1;
            }
        }
    }
    dst[code_at] = code;
    Some(out)
}

/// Decode a COBS block (without delimiter) in place. Returns the decoded
/// length, or `None` on malformed input (embedded zero, truncated block).
pub fn decode_in_place(buf: &mut [u8]) -> Option<usize> {
    let mut read = 0;
    let mut write = 0;
    while read < buf.len() {
        let code = buf[read];
        if code == 0 {
            return None;
        }
        read += 1;
        for _ in 1..code {
            if read >= buf.len() {
                return None;
            }
            if buf[read] == 0 {
                return None;
            }
            buf[write] = buf[read];
            read += 1;
            write += 1;
        }
        if code != 0xFF && read < buf.len() {
            buf[write] = 0;
            write += 1;
        }
    }
    Some(write)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;

    fn round_trip(data: &[u8]) {
        let mut enc = [0u8; 600];
        let n = encode(data, &mut enc).unwrap();
        assert!(!enc[..n].contains(&0), "encoded data must be zero-free");
        let mut dec = enc[..n].to_vec();
        let m = decode_in_place(&mut dec).unwrap();
        assert_eq!(&dec[..m], data);
    }

    #[test]
    fn classic_vectors() {
        round_trip(&[]);
        round_trip(&[0]);
        round_trip(&[0, 0]);
        round_trip(&[1, 2, 3]);
        round_trip(&[1, 0, 2]);
        round_trip(&[0xFF; 253]);
        round_trip(&[0xFF; 254]);
        round_trip(&[0xFF; 255]);
        let mixed: Vec<u8> = (0..=255u8).cycle().take(500).collect();
        round_trip(&mixed);
    }

    #[test]
    fn rejects_embedded_zero() {
        let mut buf = [2, 0, 1];
        assert_eq!(decode_in_place(&mut buf), None);
    }

    #[test]
    fn rejects_truncated_block() {
        let mut buf = [5, 1, 2];
        assert_eq!(decode_in_place(&mut buf), None);
    }
}
