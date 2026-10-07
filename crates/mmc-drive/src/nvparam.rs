//! Flash blob for the persisted parameter table — the codec only. Where the
//! blob lives and how a page is erased is the board's business
//! ([`ParamStore`]).
//!
//! Layout (little-endian u32 words): magic, version, `[f32; param::COUNT]`,
//! crc32(preceding), padded to an even word count (8-byte flash writes).

use mmc_proto::param;

const MAGIC: u32 = 0x4D4D_4350; // "MMCP"
/// Bumped whenever `param::COUNT` changes, because the blob is a flat
/// `[f32; COUNT]` and a longer table shifts the CRC word. The CRC alone would
/// reject a stale blob anyway; the version makes that a deliberate rejection
/// rather than a lucky one.
///
/// v2 (2026-08-08): added `v_dead`/`i_thresh`, 13 → 15 params. **A device
/// flashed across this boundary loads compiled-in defaults and needs its
/// profile re-applied and re-persisted.**
///
/// v3 (2026-10-01): added `hall_offset`/`hall_dir`, 15 → 17 params. Same
/// consequence: re-apply and re-persist after flashing across it.
///
/// v4 (2026-10-02): added `hall_hyst`, 17 → 18 params. Same again.
///
/// v5 (2026-10-02): added the hall position loop (`pos_kp`, `pos_ki`,
/// `pos_kd`, `pos_vmax`), `inertia` and `i_fric`, 18 → 24 params.
///
/// v6 (2026-10-06): added `ss_conduction`, 24 → 25 params.
///
/// v7 (2026-10-06): added `id_inject`, 25 → 26 params.
///
/// v8 (2026-10-06): added the hall sector widths `hall_w0`…`hall_w5`,
/// 26 → 32 params.
///
/// v9 (2026-10-06): added the i_d dither (`id_dither`, `id_dither_period`),
/// 32 → 34 params.
///
/// v10 (2026-10-07): added the position-torque feed-forward (`cog_ff`,
/// `cog_shift`, four series terms), 34 → 48 params.
const VERSION: u32 = 10;
const HDR: usize = 2; // magic + version
const CRC_IDX: usize = HDR + param::COUNT;
const WORDS: usize = (CRC_IDX + 1 + 1) & !1; // +crc, round up to even
/// Size of an encoded blob.
pub const BYTES: usize = WORDS * 4;

/// Board-side storage for the blob: one erasable flash page (or anything
/// else that survives a reset).
pub trait ParamStore {
    /// The stored bytes (at least [`BYTES`] long), blank or not.
    fn read(&mut self) -> &[u8];
    /// Erase the page, then program `blob`. May stall the CPU; the drive only
    /// calls it with the stage quiet.
    fn write(&mut self, blob: &[u8; BYTES]) -> bool;
    /// Erase only, so the next boot runs on firmware defaults.
    fn erase(&mut self) -> bool;
}

fn crc32(words: &[u32]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &word in words {
        crc ^= word;
        for _ in 0..32 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0xEDB8_8320
            } else {
                crc >> 1
            };
        }
    }
    !crc
}

/// Validate and unpack a stored blob. `None` when blank, wrong version, or
/// CRC-mismatched.
pub fn decode(bytes: &[u8]) -> Option<[f32; param::COUNT]> {
    if bytes.len() < BYTES {
        return None;
    }
    let word = |i: usize| u32::from_le_bytes(bytes[i * 4..i * 4 + 4].try_into().unwrap());
    if word(0) != MAGIC || word(1) != VERSION {
        return None;
    }
    let mut words = [0u32; CRC_IDX];
    for (i, w) in words.iter_mut().enumerate() {
        *w = word(i);
    }
    if crc32(&words) != word(CRC_IDX) {
        return None;
    }
    let mut params = [0f32; param::COUNT];
    for (i, p) in params.iter_mut().enumerate() {
        *p = f32::from_bits(words[HDR + i]);
    }
    Some(params)
}

pub fn encode(params: &[f32; param::COUNT]) -> [u8; BYTES] {
    let mut words = [0u32; WORDS];
    words[0] = MAGIC;
    words[1] = VERSION;
    for (i, v) in params.iter().enumerate() {
        words[HDR + i] = v.to_bits();
    }
    words[CRC_IDX] = crc32(&words[..CRC_IDX]);
    let mut bytes = [0u8; BYTES];
    for (i, w) in words.iter().enumerate() {
        bytes[i * 4..i * 4 + 4].copy_from_slice(&w.to_le_bytes());
    }
    bytes
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trips_and_rejects_corruption() {
        let mut p = [0f32; param::COUNT];
        for (i, v) in p.iter_mut().enumerate() {
            *v = i as f32 * 0.5 + 0.1;
        }
        let mut blob = encode(&p);
        assert_eq!(decode(&blob), Some(p));
        blob[9] ^= 1;
        assert_eq!(decode(&blob), None);
        assert_eq!(decode(&[0xFF; BYTES]), None, "erased flash is blank");
    }
}
