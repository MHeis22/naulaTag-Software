#!/bin/sh
# Build the display simulator on the host.  Run from the repo root:
#   sh tools/sim/build.sh && tools/sim/out/sim
#
# Sanitizers are used when the host toolchain supports them: the framebuffer is
# a plain array and font_draw_char() does not clip, so a string that runs off the
# panel shows up here as a clean report instead of silent corruption on the
# board.  MinGW has no ASan runtime, so it is probed for and skipped there.

set -e

OUT=tools/sim/out
mkdir -p "$OUT"

if [ -z "${SAN+x}" ]; then
    SAN="-fsanitize=address,undefined"
    echo 'int main(void){return 0;}' > "$OUT/.probe.c"
    gcc $SAN -o "$OUT/.probe.exe" "$OUT/.probe.c" >/dev/null 2>&1 || {
        SAN=""
        echo "note: toolchain has no sanitizer runtime — building without it"
    }
    rm -f "$OUT/.probe.c" "$OUT/.probe.exe"
fi

gcc -std=c11 -g -O1 -Wall -Wextra $SAN \
    -I src -I tools/sim \
    -o "$OUT/sim.exe" \
    src/display.c tools/sim/sim_stubs.c tools/sim/sim_main.c \
    -lm

echo "built $OUT/sim.exe"
