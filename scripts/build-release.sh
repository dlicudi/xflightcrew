#!/usr/bin/env bash
#
# Build the single-file release: the plugin (make -C plugin, never install),
# then xfc with the plugin built in, copied to dist/xfc. SDK=<dir> passes
# the X-Plane SDK's folder to make (default plugin/SDK).

set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"

if [[ -n "${SDK:-}" ]]; then
  make -C plugin SDK="$SDK"
else
  make -C plugin
fi

xpl="$root/plugin/build/mac_x64/xflightcrew.xpl"
XFC_PLUGIN_XPL="$xpl" cargo build --release --features embed-plugin --bin xfc

mkdir -p dist
cp target/release/xfc dist/xfc
size=$(wc -c < dist/xfc | tr -d ' ')
echo "dist/xfc: $size bytes ($(du -h dist/xfc | cut -f1)), plugin $(wc -c < "$xpl" | tr -d ' ') bytes inside"
