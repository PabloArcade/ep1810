#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-or-later
# Build ep1810dump (and ep1810dump.exe when the mingw-w64 compiler is present).
# usage: ./build.sh [output directory]      default: ./build
set -e
cd "$(dirname "$0")"
[ -f LICENSE ] || { echo "build.sh: LICENSE is missing, not building" >&2; exit 1; }
out="${1:-build}"
mkdir -p "$out"
if command -v sha256sum >/dev/null 2>&1; then sha256sum -c TABLES.sha256 >/dev/null || { echo "build.sh: a table does not match TABLES.sha256" >&2; exit 1; }; fi
${CC:-gcc} -std=c99 -O2 -Wall -Wextra -pedantic -o "$out/ep1810dump" src/ep1810dump.c
strip "$out/ep1810dump"
if command -v x86_64-w64-mingw32-gcc >/dev/null 2>&1; then
    x86_64-w64-mingw32-gcc -std=c99 -O2 -Wall -Wextra -static -Wl,--stack,33554432 -Wl,--no-insert-timestamp -s -o "$out/ep1810dump.exe" src/ep1810dump.c
fi
echo "built in $out"
