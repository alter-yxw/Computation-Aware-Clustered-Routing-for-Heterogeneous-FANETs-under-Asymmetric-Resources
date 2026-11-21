#!/usr/bin/env bash
#
# Overlay this release onto a clean ns-3.40 source tree.
#
# Usage:
#   ./apply_to_ns3.sh /path/to/ns-3.40
#
# The target must be an unpacked, unmodified ns-3.40 tree. The script applies
# the four src/ patches, then copies the contrib modules, the scratch entry
# points and the experiment scripts into the tree. It never overwrites an
# existing contrib module and never touches build artefacts.
#
# WARNING: the target is modified in place and there is no automatic rollback.
# Back up the original ns-3.40 tree first, or run "git init" inside it before
# applying so that "git checkout -- ." can restore it.

set -euo pipefail

ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
TARGET=${1:-}

if [[ -z $TARGET ]]; then
    echo "usage: $0 /path/to/ns-3.40" >&2
    exit 2
fi
if [[ ! -f $TARGET/ns3 || ! -f $TARGET/VERSION ]]; then
    echo "error: $TARGET is not an ns-3 source tree" >&2
    exit 2
fi
if [[ $(tr -d '[:space:]' < "$TARGET/VERSION") != "3.40" ]]; then
    echo "error: this release targets ns-3.40; found $(cat "$TARGET/VERSION")" >&2
    exit 2
fi

for module in cafbcr fanet qrcf icra cfr mognn; do
    if [[ -e $TARGET/contrib/$module ]]; then
        echo "error: $TARGET/contrib/$module already exists; use a clean ns-3.40 tree" >&2
        exit 2
    fi
done

echo "Applying ns-3.40 patches to $TARGET"
for patch_file in "$ROOT"/patches/ns-3.40/*.patch; do
    if [[ ${SKIP_ARP_PATCH:-0} == 1 && $(basename "$patch_file") == 0001-* ]]; then
        echo "  skipping $(basename "$patch_file") (SKIP_ARP_PATCH=1)"
        continue
    fi
    echo "  $(basename "$patch_file")"
    if ! patch --forward --silent -p1 -d "$TARGET" < "$patch_file"; then
        echo "error: failed to apply $(basename "$patch_file")" >&2
        exit 1
    fi
done

echo "Copying CA-FBCR modules and entry points"
cp -a "$ROOT"/contrib/. "$TARGET/contrib/"
cp -a "$ROOT"/scratch/. "$TARGET/scratch/"
cp -a "$ROOT"/scripts/*.sh "$TARGET/"

cat <<EOF

Done. Build with:

  cd $TARGET
  ./ns3 configure --build-profile=default --enable-asserts --enable-logs \\
      --disable-examples --disable-tests \\
      --enable-modules='cafbcr;fanet;qrcf;cfr;icra;mognn;applications;internet;wifi;mobility;energy;flow-monitor;olsr;aodv' \\
      -- -DNS3_FETCH_OPTIONAL_COMPONENTS=OFF -DNS3_CCACHE=OFF
  ./ns3 build ca-fbcr-test -j2

See README.md for run commands and scenario details.

Note: $TARGET was modified in place. Keep the original ns-3.40 copy for rollback.
EOF
