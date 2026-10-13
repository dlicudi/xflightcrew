//! The EFB link for the sim panel's Settings tab (2026-10-13: the pilot
//! wanted the link reachable from the sim, where only the terminal and the
//! log had it): the URL as text and its QR code as a module matrix, one
//! payload on `xcopilot/status/efb_url`.
//!
//! The matrix is made HERE, not in the plugin: the `qrcode` crate (MIT OR
//! Apache-2.0, no dependencies with its default features off) is a tested
//! encoder, the plugin stays a dumb painter of rectangles, and the
//! encoding is checked by a decode round trip in a plain `cargo test`.
//!
//! Payload: `url|N|hex`, or empty while the EFB is off or not yet
//! listening. `N` is the side in modules INCLUDING the four-module quiet
//! zone; `hex` is the N*N modules row-major, dark = 1, most significant
//! bit first, the last nibble padded with zeros. The URL never holds a
//! '|' (an IP, a port, a token of letters, digits, '-' or '_').

use qrcode::{Color, EcLevel, QrCode};

/// The quiet zone the QR specification asks for, in modules.
pub(crate) const QUIET: usize = 4;

/// The module matrix of `url` at error correction M, with its quiet zone:
/// the side and the modules row-major (true = dark). `None` if it will not
/// encode (never for a URL of the EFB's length).
pub(crate) fn matrix(url: &str) -> Option<(usize, Vec<bool>)> {
    let code = QrCode::with_error_correction_level(url.as_bytes(), EcLevel::M).ok()?;
    let w = code.width();
    let colors = code.to_colors();
    let n = w + 2 * QUIET;
    let mut out = vec![false; n * n];
    for y in 0..w {
        for x in 0..w {
            out[(y + QUIET) * n + x + QUIET] = colors[y * w + x] == Color::Dark;
        }
    }
    Some((n, out))
}

/// The panel payload for the EFB link now (`None` = off or not listening).
pub(crate) fn panel_payload(url: Option<&str>) -> String {
    let Some(url) = url.filter(|u| !u.is_empty() && !u.contains(['|', '\n'])) else {
        return String::new();
    };
    let Some((n, modules)) = matrix(url) else {
        return String::new();
    };
    format!("{url}|{n}|{}", pack_hex(&modules))
}

/// Modules to hex, four per digit, most significant first.
fn pack_hex(bits: &[bool]) -> String {
    bits.chunks(4)
        .map(|c| {
            let v = c
                .iter()
                .enumerate()
                .fold(0u32, |acc, (i, &b)| acc | (u32::from(b) << (3 - i)));
            char::from_digit(v, 16).unwrap_or('0')
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    const URL: &str = "http://192.168.1.20:8787/?t=3f9a0c1d2b7e4a6f8c5d1e0b9a7f6c3d";

    /// Read a payload back as the plugin does: url, side, modules.
    fn unpack(payload: &str) -> (String, usize, Vec<bool>) {
        let mut f = payload.split('|');
        let url = f.next().unwrap().to_string();
        let n: usize = f.next().unwrap().parse().unwrap();
        let hex = f.next().unwrap();
        assert!(f.next().is_none(), "three fields");
        assert_eq!(hex.len(), (n * n).div_ceil(4));
        let bits: Vec<bool> = hex
            .chars()
            .flat_map(|c| {
                let v = c.to_digit(16).unwrap();
                (0..4).map(move |i| v & (8 >> i) != 0)
            })
            .take(n * n)
            .collect();
        (url, n, bits)
    }

    #[test]
    fn off_or_not_listening_is_an_empty_payload() {
        assert_eq!(panel_payload(None), "");
        assert_eq!(panel_payload(Some("")), "");
        assert_eq!(
            panel_payload(Some("http://x|y")),
            "",
            "a '|' would split the payload"
        );
    }

    #[test]
    fn the_payload_carries_the_whole_link_with_its_token() {
        let p = panel_payload(Some(URL));
        let (url, n, bits) = unpack(&p);
        assert_eq!(url, URL);
        assert!(url.contains("?t=3f9a0c1d2b7e4a6f8c5d1e0b9a7f6c3d"));
        assert_eq!(bits.len(), n * n);
        // Fits the plugin's 2 KB slot with room, even for a 64 char token.
        let long = format!("http://192.168.100.200:65535/?t={}", "a".repeat(64));
        assert!(panel_payload(Some(&long)).len() < crate::plugin_status::BRIEF_MAX);
    }

    #[test]
    fn the_matrix_has_its_quiet_zone_and_a_real_version() {
        let (_, n, bits) = unpack(&panel_payload(Some(URL)));
        let w = n - 2 * QUIET;
        assert_eq!((w - 17) % 4, 0, "a QR side is 17 + 4 x version");
        assert!((21..=57).contains(&w), "version 1 to 10, got side {w}");
        for y in 0..n {
            for x in 0..n {
                let inside = (QUIET..n - QUIET).contains(&x) && (QUIET..n - QUIET).contains(&y);
                if !inside {
                    assert!(!bits[y * n + x], "quiet zone dark at {x},{y}");
                }
            }
        }
        // The finder's corner module is dark.
        assert!(bits[QUIET * n + QUIET]);
        // A generated token's link (10 digits) is version 3: the biggest
        // modules at panel size.
        let short = format!("http://192.168.1.20:8787/?t={}", "3f9a0c1d2b");
        assert_eq!(matrix(&short).map(|(n, _)| n), Some(29 + 2 * QUIET));
    }

    #[test]
    fn the_published_matrix_decodes_back_to_the_link() {
        let (_, n, bits) = unpack(&panel_payload(Some(URL)));
        // Painted as the plugin does: dark on light, a few pixels a module.
        let px = 6;
        let mut img = rqrr::PreparedImage::prepare_from_greyscale(n * px, n * px, |x, y| {
            if bits[(y / px) * n + x / px] {
                0
            } else {
                255
            }
        });
        let grids = img.detect_grids();
        assert_eq!(grids.len(), 1);
        let (meta, text) = grids[0].decode().unwrap();
        assert_eq!(text, URL);
        assert_eq!(meta.ecc_level, 0, "rqrr's 0 is level M");
    }
}
