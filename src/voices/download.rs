//! Installing and removing a voice or speech model: the one place `xfc
//! voices` touches the models folder or the network.
//!
//! A download is atomic: each file streams to `<name>.part`, its size AND
//! checksum are checked, and only when every file of the entry passed are
//! they renamed in. A failure or interruption leaves only `.part` files,
//! which nothing reads as installed and the next run removes.

use std::collections::BTreeMap;
use std::io::{Read, Write};
use std::path::Path;
use std::time::Duration;

use anyhow::{anyhow, Context, Result};
use md5::Digest as _;

use super::catalogue::{self, Digest, Entry, Kind, RemoteFile};

/// Progress through one entry's files.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Progress {
    /// Which file, from 0, of `files`.
    pub file: usize,
    pub files: usize,
    /// Bytes done and expected across the whole entry.
    pub done: u64,
    pub total: u64,
}

impl Progress {
    pub fn percent(&self) -> u8 {
        if self.total == 0 {
            return 0;
        }
        ((self.done.saturating_mul(100)) / self.total).min(100) as u8
    }
}

/// The folder's listing, `name -> bytes`, regular files only. A missing
/// folder is an empty one.
pub fn list_dir(dir: &Path) -> BTreeMap<String, u64> {
    std::fs::read_dir(dir)
        .into_iter()
        .flatten()
        .flatten()
        .filter_map(|e| {
            let meta = e.metadata().ok()?;
            meta.is_file()
                .then(|| (e.file_name().to_string_lossy().into_owned(), meta.len()))
        })
        .collect()
}

/// Remove leftover `.part` files from an interrupted run.
pub fn clean_stale_parts(dir: &Path) -> Vec<String> {
    let stale = catalogue::stale_parts(&list_dir(dir));
    for name in &stale {
        let _ = std::fs::remove_file(dir.join(name));
    }
    stale
}

fn http() -> Result<reqwest::blocking::Client> {
    reqwest::blocking::Client::builder()
        .connect_timeout(Duration::from_secs(15))
        // A whole model can take minutes: no total timeout (the blocking
        // client's default is 30 s).
        .timeout(None)
        .user_agent(concat!("xflightcrew/", env!("XVL_VERSION")))
        .build()
        .context("building the HTTP client")
}

/// GET a URL as a body to stream. Blocking: call off the UI thread and
/// never inside the async runtime.
/// A busy server (429, 5xx) is asked again twice, a little later each time.
pub fn open_url(url: &str) -> Result<Box<dyn Read + Send>> {
    let client = http()?;
    let mut attempt = 0;
    loop {
        let resp = client
            .get(url)
            .send()
            .with_context(|| format!("fetching {url}"))?;
        let status = resp.status();
        if status.is_success() {
            return Ok(Box::new(resp));
        }
        let busy = status.as_u16() == 429 || status.is_server_error();
        if !busy || attempt >= 2 {
            return Err(anyhow!("fetching {url}: HTTP {status}"));
        }
        attempt += 1;
        std::thread::sleep(Duration::from_secs(2 * attempt));
    }
}

/// Fetch a small text resource whole (the catalogue).
pub fn fetch_text(url: &str) -> Result<String> {
    let mut s = String::new();
    open_url(url)?
        .read_to_string(&mut s)
        .with_context(|| format!("reading {url}"))?;
    Ok(s)
}

enum Hasher {
    Md5(md5::Md5),
    Sha256(sha2::Sha256),
}

impl Hasher {
    fn for_digest(d: &Digest) -> Self {
        match d {
            Digest::Md5(_) => Hasher::Md5(md5::Md5::new()),
            Digest::Sha256(_) => Hasher::Sha256(sha2::Sha256::new()),
        }
    }
    fn update(&mut self, b: &[u8]) {
        match self {
            Hasher::Md5(h) => h.update(b),
            Hasher::Sha256(h) => h.update(b),
        }
    }
    fn hex(self) -> String {
        let bytes: Vec<u8> = match self {
            Hasher::Md5(h) => h.finalize().to_vec(),
            Hasher::Sha256(h) => h.finalize().to_vec(),
        };
        bytes.iter().map(|b| format!("{b:02x}")).collect()
    }
}

/// Stream one body into `part`, hashing as it goes; Ok only when the size and
/// the checksum both match. `on_bytes` is told each chunk's length.
pub fn stream_verified(
    mut body: impl Read,
    part: &Path,
    file: &RemoteFile,
    on_bytes: &mut dyn FnMut(u64),
) -> Result<()> {
    let mut out =
        std::fs::File::create(part).with_context(|| format!("creating {}", part.display()))?;
    let mut hasher = Hasher::for_digest(&file.digest);
    let mut buf = vec![0u8; 256 * 1024];
    let mut size: u64 = 0;
    loop {
        let n = body
            .read(&mut buf)
            .with_context(|| format!("downloading {}", file.local_name))?;
        if n == 0 {
            break;
        }
        out.write_all(&buf[..n])
            .with_context(|| format!("writing {}", part.display()))?;
        hasher.update(&buf[..n]);
        size += n as u64;
        if size > file.size {
            return Err(anyhow!(
                "{}: more than the expected {} bytes",
                file.local_name,
                file.size
            ));
        }
        on_bytes(n as u64);
    }
    out.flush()?;
    out.sync_all().ok();
    if size != file.size {
        return Err(anyhow!(
            "{}: got {size} bytes, expected {}",
            file.local_name,
            file.size
        ));
    }
    let got = hasher.hex();
    if got != file.digest.hex() {
        return Err(anyhow!(
            "{}: {} mismatch (got {got}, expected {})",
            file.local_name,
            file.digest.name(),
            file.digest.hex()
        ));
    }
    Ok(())
}

/// Install an entry into `dir`, every file fetched through `open`. Either all
/// of its files are renamed in, verified, or none is: a failure removes the
/// parts and anything already renamed.
pub fn install_with(
    entry: &Entry,
    dir: &Path,
    open: &dyn Fn(&RemoteFile) -> Result<Box<dyn Read + Send>>,
    progress: &mut dyn FnMut(Progress),
) -> Result<()> {
    if entry.is_local() {
        return Err(anyhow!("{} is local: nothing to download", entry.key));
    }
    std::fs::create_dir_all(dir).with_context(|| format!("creating {}", dir.display()))?;
    let total = entry.download_size();
    let files = entry.files.len();
    let parts: Vec<std::path::PathBuf> = entry
        .files
        .iter()
        .map(|f| dir.join(format!("{}.part", f.local_name)))
        .collect();
    let cleanup = |parts: &[std::path::PathBuf]| {
        for p in parts {
            let _ = std::fs::remove_file(p);
        }
    };
    let mut done: u64 = 0;
    progress(Progress {
        file: 0,
        files,
        done,
        total,
    });
    for (i, f) in entry.files.iter().enumerate() {
        let result = open(f).and_then(|body| {
            stream_verified(body, &parts[i], f, &mut |n| {
                done += n;
                progress(Progress {
                    file: i,
                    files,
                    done,
                    total,
                });
            })
        });
        if let Err(e) = result {
            cleanup(&parts);
            return Err(e);
        }
    }
    // Every file verified: rename in. The model goes last, so a crash between
    // renames leaves no `.onnx` without its config (and installed needs both).
    let mut renamed: Vec<std::path::PathBuf> = Vec::new();
    for (i, f) in entry.files.iter().enumerate().rev() {
        let target = dir.join(&f.local_name);
        if let Err(e) = std::fs::rename(&parts[i], &target) {
            cleanup(&parts);
            for t in &renamed {
                let _ = std::fs::remove_file(t);
            }
            return Err(anyhow!("installing {}: {e}", target.display()));
        }
        renamed.push(target);
    }
    Ok(())
}

/// Install an entry from its own URLs.
pub fn install(entry: &Entry, dir: &Path, progress: &mut dyn FnMut(Progress)) -> Result<()> {
    install_with(entry, dir, &|f| open_url(&f.url), progress)
}

/// Remove an entry's files from `dir` (the gate is the caller's).
pub fn remove(kind: Kind, key: &str, dir: &Path) -> Result<()> {
    // The model first, so a half-done remove reads as not installed.
    for name in catalogue::local_names(kind, key) {
        let path = dir.join(&name);
        match std::fs::remove_file(&path) {
            Ok(()) => {}
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
            Err(e) => return Err(anyhow!("removing {}: {e}", path.display())),
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scratch(tag: &str) -> std::path::PathBuf {
        let d = std::env::temp_dir().join(format!(
            "xfc-voices-test-{tag}-{}-{:?}",
            std::process::id(),
            std::thread::current().id()
        ));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(&d).unwrap();
        d
    }

    fn md5_hex(b: &[u8]) -> String {
        let mut h = Hasher::for_digest(&Digest::Md5(String::new()));
        h.update(b);
        h.hex()
    }

    fn file(name: &str, body: &[u8]) -> RemoteFile {
        RemoteFile {
            url: format!("test://{name}"),
            local_name: name.into(),
            size: body.len() as u64,
            digest: Digest::Md5(md5_hex(body)),
        }
    }

    fn entry(files: Vec<RemoteFile>) -> Entry {
        Entry {
            key: "xx_XX-test-medium".into(),
            kind: Kind::Piper,
            country: String::new(),
            lang: "xx_XX".into(),
            quality: "medium".into(),
            speakers: 1,
            files,
        }
    }

    const MODEL: &[u8] = b"pretend onnx weights";
    const CONFIG: &[u8] = b"{\"audio\":{}}";

    fn opener(
        bodies: Vec<(&'static str, &'static [u8])>,
    ) -> impl Fn(&RemoteFile) -> Result<Box<dyn Read + Send>> {
        move |f| {
            let b = bodies
                .iter()
                .find(|(n, _)| *n == f.local_name)
                .map(|(_, b)| *b)
                .ok_or_else(|| anyhow!("connection refused"))?;
            Ok(Box::new(std::io::Cursor::new(b)) as Box<dyn Read + Send>)
        }
    }

    #[test]
    fn a_verified_download_installs_both_files_and_leaves_no_part() {
        let dir = scratch("ok");
        let e = entry(vec![
            file("xx_XX-test-medium.onnx", MODEL),
            file("xx_XX-test-medium.onnx.json", CONFIG),
        ]);
        let open = opener(vec![
            ("xx_XX-test-medium.onnx", MODEL),
            ("xx_XX-test-medium.onnx.json", CONFIG),
        ]);
        let mut last = None;
        install_with(&e, &dir, &open, &mut |p| last = Some(p)).unwrap();
        let listing = list_dir(&dir);
        assert_eq!(
            listing.keys().collect::<Vec<_>>(),
            ["xx_XX-test-medium.onnx", "xx_XX-test-medium.onnx.json"]
        );
        assert_eq!(last.unwrap().percent(), 100);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_bad_checksum_installs_neither_file() {
        let dir = scratch("bad");
        let mut cfg = file("xx_XX-test-medium.onnx.json", CONFIG);
        cfg.digest = Digest::Md5("0".repeat(32));
        let e = entry(vec![file("xx_XX-test-medium.onnx", MODEL), cfg]);
        let open = opener(vec![
            ("xx_XX-test-medium.onnx", MODEL),
            ("xx_XX-test-medium.onnx.json", CONFIG),
        ]);
        let err = install_with(&e, &dir, &open, &mut |_| {}).unwrap_err();
        assert!(err.to_string().contains("md5 mismatch"), "{err}");
        assert!(list_dir(&dir).is_empty(), "{:?}", list_dir(&dir));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_short_or_failed_download_leaves_nothing_that_looks_installed() {
        let dir = scratch("short");
        let mut model = file("xx_XX-test-medium.onnx", MODEL);
        model.size += 10;
        let e = entry(vec![model, file("xx_XX-test-medium.onnx.json", CONFIG)]);
        let open = opener(vec![("xx_XX-test-medium.onnx", MODEL)]);
        let err = install_with(&e, &dir, &open, &mut |_| {}).unwrap_err();
        assert!(err.to_string().contains("expected"), "{err}");
        assert!(list_dir(&dir).is_empty());

        // The second file's fetch failing removes the verified first part too.
        let e = entry(vec![
            file("xx_XX-test-medium.onnx", MODEL),
            file("xx_XX-test-medium.onnx.json", CONFIG),
        ]);
        let err = install_with(
            &e,
            &dir,
            &opener(vec![("xx_XX-test-medium.onnx", MODEL)]),
            &mut |_| {},
        )
        .unwrap_err();
        assert!(err.to_string().contains("refused"), "{err}");
        assert!(list_dir(&dir).is_empty());
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn stale_parts_are_cleaned_and_remove_takes_both_files() {
        let dir = scratch("clean");
        std::fs::write(dir.join("a.onnx.part"), b"x").unwrap();
        std::fs::write(dir.join("k.onnx"), b"x").unwrap();
        std::fs::write(dir.join("k.onnx.json"), b"x").unwrap();
        assert_eq!(clean_stale_parts(&dir), ["a.onnx.part"]);
        remove(Kind::Piper, "k", &dir).unwrap();
        assert!(list_dir(&dir).is_empty());
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn sha256_is_checked_for_whisper() {
        let mut h = Hasher::for_digest(&Digest::Sha256(String::new()));
        h.update(b"abc");
        assert_eq!(
            h.hex(),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
    }
}
