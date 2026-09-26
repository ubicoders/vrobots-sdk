#!/usr/bin/env bash
#
# Download the VRobots SDK C bundle that matches this checkout, verify it and
# unpack it where the C++ examples' CMake build finds it on its own.
#
#   bash scripts/get_cpp_bundle.sh            # the version of this checkout
#   bash scripts/get_cpp_bundle.sh 0.1.11     # a specific version
#
# What it does:
#   1. reads the SDK version from crates/vrobots-sdk-sys/Cargo.toml (or takes
#      the one given on the command line);
#   2. downloads vrobots_sdk-cpp-<version>-linux-x86_64.tar.gz and SHA256SUMS
#      from https://github.com/ubicoders/vrobots-sdk/releases/tag/v<version>;
#   3. checks the archive against SHA256SUMS;
#   4. unpacks it into ../vrobots_sdk-cpp-<version>-linux-x86_64/ next to this
#      repository, the folder examples/cpp/CMakeLists.txt looks for, and
#      removes the archive.
#
# Then build and run the examples from the repository root:
#
#   cmake -S examples/cpp -B target/cpp-build -DCMAKE_BUILD_TYPE=Release
#   cmake --build target/cpp-build --config Release
#   ./target/cpp-build/ex01_hello_states
#
# Linux x86-64 only (glibc 2.28 or newer). Windows: scripts/get_cpp_bundle.ps1.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OS=linux-x86_64

if [ "$(uname -s)" != "Linux" ] || [ "$(uname -m)" != "x86_64" ]; then
    echo "get_cpp_bundle.sh: this script is for Linux x86-64; on Windows run scripts/get_cpp_bundle.ps1. No other platform has a build." >&2
    exit 1
fi

if [ $# -ge 1 ]; then
    VERSION=${1#v}
else
    VERSION=$(sed -n 's/^version *= *"\([^"]*\)".*/\1/p' "$REPO_ROOT/crates/vrobots-sdk-sys/Cargo.toml" | head -1)
fi
[ -n "$VERSION" ] || { echo "get_cpp_bundle.sh: could not read the version; pass it as the first argument" >&2; exit 1; }

ARCHIVE="vrobots_sdk-cpp-${VERSION}-${OS}.tar.gz"
BASE_URL="https://github.com/ubicoders/vrobots-sdk/releases/download/v${VERSION}"
DEST="$REPO_ROOT/../vrobots_sdk-cpp-${VERSION}-${OS}"

if [ -f "$DEST/include/vrobots_sdk.h" ] && [ -f "$DEST/lib/libvrobots_sdk_capi.so" ]; then
    echo "already unpacked: $DEST"
    exit 0
fi

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
cd "$WORK"

echo "downloading $ARCHIVE from $BASE_URL"
curl -fsSL --retry 3 -o "$ARCHIVE" "$BASE_URL/$ARCHIVE" \
    || { echo "get_cpp_bundle.sh: download failed. Does https://github.com/ubicoders/vrobots-sdk/releases/tag/v${VERSION} exist?" >&2; exit 1; }
curl -fsSL --retry 3 -o SHA256SUMS "$BASE_URL/SHA256SUMS"

echo "verifying"
sha256sum -c SHA256SUMS --ignore-missing --quiet

mkdir -p "$DEST"
tar -xzf "$ARCHIVE" -C "$DEST"
DEST="$(cd "$DEST" && pwd)"

cat <<MSG

unpacked SDK ${VERSION} into $DEST
  include/vrobots_sdk.h, include/vrobots_sdk.hpp, lib/libvrobots_sdk_capi.so

Build and run the examples from the repository root:

  cmake -S examples/cpp -B target/cpp-build -DCMAKE_BUILD_TYPE=Release
  cmake --build target/cpp-build --config Release
  ./target/cpp-build/ex01_hello_states

For your own project, add $DEST/include to the include path and link
$DEST/lib/libvrobots_sdk_capi.so.
MSG
