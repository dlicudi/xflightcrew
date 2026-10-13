//! `xfc licenses`: the notices a single shipped binary carries, embedded
//! VERBATIM from the files in the repo (never paraphrased): the program's
//! GPL-3.0 notice (`licenses/NOTICE.txt`, the full text with `--full`) and
//! every third-party part in the binary or the plugin it carries. The
//! X-Plane SDK's text is `licenses/X-Plane-SDK.txt` (the SDK itself is not
//! in the repo); the rest stay where their code lives.

/// (name, what it is here, the licence text).
pub const THIRD_PARTY: &[(&str, &str, &str)] = &[
    (
        "X-Plane SDK",
        "the plugin's link to X-Plane (MIT/BSD)",
        include_str!("../licenses/X-Plane-SDK.txt"),
    ),
    (
        "Dear ImGui",
        "the plugin's panel (MIT)",
        include_str!("../plugin/third_party/imgui/LICENSE.txt"),
    ),
    (
        "XPMP2",
        "the plugin's traffic drawing (MIT)",
        include_str!("../plugin/third_party/XPMP2/LICENSE"),
    ),
    (
        "B612",
        "the panel's font (SIL Open Font License 1.1)",
        include_str!("../plugin/third_party/b612/OFL.txt"),
    ),
    (
        "AFV-Native",
        "the radio's squelch click and crackle (BSD-3-Clause)",
        include_str!("../assets/afv/AFV_COPYING.md"),
    ),
];

/// The program's notice.
pub const NOTICE: &str = include_str!("../licenses/NOTICE.txt");
/// The GNU General Public License, version 3.
pub const GPL: &str = include_str!("../LICENSE");

/// The whole printout: the notice, then each third-party text; `full`
/// adds the GPL's text.
pub fn text(full: bool) -> String {
    let rule = "=".repeat(72);
    let mut out = String::from(NOTICE);
    if full {
        out += &format!("\n{rule}\n\n{GPL}");
    } else {
        out += "\nThe full GNU General Public License: xfc licenses --full\n";
    }
    out += &format!("\n{rule}\nThird-party parts, each under its own licence:\n{rule}\n");
    for (name, what, body) in THIRD_PARTY {
        out += &format!(
            "\n{name}: {what}\n{}\n\n{}\n",
            "-".repeat(72),
            body.trim_end()
        );
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_notice_is_carried_verbatim() {
        let t = text(false);
        assert!(t.contains("GNU General Public License as published by"));
        for (name, _, body) in THIRD_PARTY {
            assert!(t.contains(name));
            assert!(
                body.len() > 200,
                "{name}'s text is too short to be a licence"
            );
            assert!(t.contains(body.trim_end()));
        }
        assert!(text(true).contains("TERMS AND CONDITIONS"));
        assert!(!t.contains("TERMS AND CONDITIONS"));
    }
}
