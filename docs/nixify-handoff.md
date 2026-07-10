# Handoff prompt — nixify phase 1

Paste everything below into a fresh agent session in this repo.

---

Execute phase 1 of `docs/nixify-plan.md`: replace PlatformIO with a nix
flake building this ESP32 firmware via ESP-IDF 4.4 + arduino-esp32 2.0.17
as an IDF component.

## Read first (in this order)

1. `docs/nixify-plan.md` — work packages WP1–WP7, acceptance criteria,
   risks. This is the authoritative task list.
2. `docs/refactoring-roadmap.md` — Decision log Q1–Q11. These decisions
   are settled; do not relitigate them.
3. `AGENTS.md` — repo conventions (flashing, board registry).

## Hard constraints

- **Versions are pinned by decision Q6**: arduino-esp32 2.0.17, ESP-IDF
  4.4.x, xtensa gcc 8.4.0. If IDF 4.4 proves unobtainable via nix at
  reasonable cost, STOP and ask the user — do not silently target 5.x.
- **No secrets in the nix store** (decision Q3). Station identity is
  runtime NVS; `include/factory_defaults.h` stays secret-free and loses
  its `ENABLE_*` block once variants inject the flags.
- **Clean cutover** (decision Q8): `platformio.ini` and every `pio`
  command in docs die in the same commit the flake lands. No dual build
  systems on HEAD.
- Do not move `src/*.cpp`; the IDF `main/` component points at them.
- All Arduino libraries are flake inputs with `flake = false` (Q7); no
  vendoring, no runtime downloads.
- Reference numbers for parity: last PlatformIO mvp build was Flash
  1,085,725 B (55.2% of 0x1E0000 app partition), RAM 15.5%. The nix-built
  mvp image must land within ±5%.

## Execution style

- **Delegate all implementation to subagents** (`task` tool,
  `isolated: true` for file-editing tasks); you orchestrate, review, and
  merge. Launch independent work packages in parallel — after WP1
  (toolchain pin) completes, WP2 (IDF scaffolding) and WP3 (library
  components) run concurrently; WP5 (flash tooling), WP6 (doc cutover),
  and WP7 (CI) run concurrently once the mvp variant builds.
- Subagents run no version control; you own the jj workflow and merge
  their parked refs deliberately.
- Each subagent assignment must name exact files, the acceptance check,
  and forbid running project-wide formatters/test suites.

## Verification before yielding

- `nix build .#firmware-mvp .#firmware-airquality-sps30
  .#firmware-airquality-sds011` succeed.
- mvp bin size within the parity band.
- `nix flake check` green; `.github/workflows/ci.yml` runs it.
- `grep -ri platformio` finds nothing outside jj history and
  `docs/refactoring-roadmap.md` / `docs/nixify-plan.md` prose.
- Report the flash command for the user's manual hardware parity check
  (`nix run .#flash-mvp` + expected serial output); phase 1 sign-off is
  theirs after that check.

## Out of scope

QEMU (phase 2), component extraction / Catch2 tests (phase 3), arduino-
esp32 3.x upgrade, pure-IDF migration, enclosure/ and 3d_files/.
