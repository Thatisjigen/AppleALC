#!/bin/bash
# build.sh <Lilu DEBUG kext (its Contents/Resources holds the SDK)> <MacKernelSDK checkout> <out dir>  ->  <out>/IDispHDA.kext
set -euo pipefail
cd "$(dirname "$0")"
LILU=$1; MKSDK=$2; OUT=$3; SDK="$LILU/Contents/Resources"
[ -f "$SDK/Headers/plugin_start.hpp" ] || { echo "STOP: no Lilu SDK in $LILU (use the DEBUG Lilu.kext)"; exit 1; }
K="$MKSDK/Headers"; [ -d "$K" ] || { echo "STOP: no MacKernelSDK at $MKSDK"; exit 1; }

# The numbers first: they are plain code, and a mistake here costs a boot to find.
mkdir -p "$OUT"
xcrun clang++ -std=c++17 -Wall -Wextra -Werror test_tables.cpp -o "$OUT/test_tables"
"$OUT/test_tables"

mkdir -p "$OUT/IDispHDA.kext/Contents/MacOS"
FLAGS=(-arch x86_64 -mmacosx-version-min=15.0 -fapple-kext -mkernel -nostdinc -I"$K" -I"$SDK"
       -DKERNEL -DKERNEL_PRIVATE -DDRIVER_PRIVATE -DAPPLE -DNeXT -DPRODUCT_NAME=IDispHDA -DMODULE_VERSION=1.0.0
       -std=c++17 -fno-rtti -fno-exceptions -fno-builtin -fno-common -O2 -Wall -Wno-unused-parameter -g0)
xcrun clang++ "${FLAGS[@]}" -c idisphda.cpp -o "$OUT/idisphda.o"
# Lilu's bundled startup source carries its own conversion and shadow warnings.
xcrun clang++ "${FLAGS[@]}" -Wno-sign-conversion -Wno-shadow -c "$SDK/Library/plugin_start.cpp" -o "$OUT/plugin_start.o"
xcrun clang++ -arch x86_64 -mmacosx-version-min=15.0 -fapple-kext -nostdlib -Xlinker -kext -L"$MKSDK/Library/x86_64" -lkmodc++ -lkmod -lcc_kext \
  "$OUT/idisphda.o" "$OUT/plugin_start.o" -o "$OUT/IDispHDA.kext/Contents/MacOS/IDispHDA"
cp Info.plist "$OUT/IDispHDA.kext/Contents/Info.plist"
BIN="$OUT/IDispHDA.kext/Contents/MacOS/IDispHDA"
plutil -lint "$OUT/IDispHDA.kext/Contents/Info.plist" >/dev/null
otool -hv "$BIN" | grep -q KEXTBUNDLE || { echo "STOP: not a kext bundle Mach-O"; exit 1; }
nm -m "$BIN" | grep -q ' _kmod_info$' || { echo "STOP: no _kmod_info"; exit 1; }
echo "IDispHDA $(shasum -a 256 "$BIN" | cut -c1-16) $(stat -f %z "$BIN") B"
