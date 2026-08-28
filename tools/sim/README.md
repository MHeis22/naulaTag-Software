# Display simulator

Renders the real screen layout on a PC, so `src/display.c` can be iterated on
without flashing the board.

`src/display.c` is compiled **verbatim** — same font, same coordinates, same
1 bpp framebuffer. Only the panel side is replaced: `sim_stubs.c` stands in for
`epd_cog.c` and `epd_hal.c` and captures the frame the COG driver would have
shifted out. What you see is what the panel gets, pixel for pixel; the only
things not modelled are the waveform itself (ghosting, greys during refresh)
and panel contrast.

## Use

From PowerShell — note it is `.\tools\sim\build.ps1`, not `sh`. PowerShell has
no `sh`, and its `bash` is WSL's, which cannot see the MSYS2 toolchain:

```bash
.\tools\sim\build.ps1
```

```bash
.\tools\sim\out\sim.exe
```

From an MSYS2 / WSL / Linux shell, `sh tools/sim/build.sh` does the same.
Either way, one PNG per scenario lands in `tools/sim/out/`.

```bash
.\tools\sim\out\sim.exe room --ascii
```

Renders a single scenario and also dumps it to the terminal as half-blocks, for
a quick look without leaving the shell.

```bash
.\tools\sim\out\sim.exe custom --temp -3.7 --humid 91.2 --volt 2.85
```

Renders arbitrary values. `--scale N` sets pixel magnification (default 4),
`--out DIR` the output directory.

## Physical scale

The panel is small — 46.5 x 24 mm of viewing area for 248 x 128 px, so
5.33 px/mm, about 135 dpi. That is easy to forget when judging a layout at 4x
on a monitor, so the geometry from the datasheet is baked in:

- the module is drawn to scale, 57.75 x 29.5 mm including the bezel
- PNGs carry a `pHYs` chunk, so printing one at 100% gives a life-size mockup
  you can hold against the enclosure
- the tool reports text heights in mm: at the current scales, temperature is
  5.2 mm tall, humidity 3.9 mm, voltage 2.6 mm

The panel is rated -15 to +60 C. Any scenario whose temperature lands outside
that is flagged, since the waveform is not specified there.

## Scenarios

| name | what it exercises |
|---|---|
| `room`  | typical indoor reading |
| `cold`  | negative temperature, minus sign |
| `hot`   | widest digits, low humidity |
| `boot`  | first update, no history yet |
| `one`   | single history sample |
| `few`   | five samples, sparse graph |
| `flat`  | constant history — degenerate min == max range |
| `spike` | out-of-range excursions, graph clamping |
| `wide`  | widest strings the format strings can produce |
| `low`   | depleted battery |

Add cases to the `scenarios[]` table in `sim_main.c`. The edge cases are the
point: `boot`, `one`, `flat` and `wide` are the ones that are awkward to
reproduce on real hardware but trivial here.

Each line of output also reports the update mode (`fast` / `normal`), the value
handed to the panel's temperature register, and ink coverage — a rough proxy
for refresh cost.

## Note on bounds checking

`font_draw_char()` in `src/font5x7.h` does not clip to the framebuffer, so a
string wider than the panel corrupts memory past `fb[]` rather than being cut
off. `build.sh` enables ASan/UBSan when the host toolchain has them, which turns
that into a clean report. MinGW ships no sanitizer runtime, so on plain MSYS2
the build silently drops the flag — run the simulator under WSL or Linux when
adding new layout code:

```bash
sh tools/sim/build.sh && tools/sim/out/sim
```
