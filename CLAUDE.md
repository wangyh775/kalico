# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Kalico is a community-maintained fork of Klipper 3D printer firmware. It adds features and behavior beyond mainline Klipper (see `docs/Kalico_Additions.md`). The repo contains both host-side Python code and MCU firmware C code — many changes require checking both sides.

## Development Commands

### Python Environment
- Python >= 3.9 (`pyproject.toml`)
- Install deps: `uv sync --dev`

### Lint & Format
- `uv run ruff check .` — lint
- `uv run ruff format .` — format
- Pre-commit hooks: `uv run pre-commit run --all-files`

### Tests
- Host pytest: `uv run pytest`
- Single file: `uv run pytest test/test_autososave.py`
- Single test: `uv run pytest test/test_autosave.py::test_autosave_includes`
- Klippy regression (requires MCU dicts): `uv run pytest test/klippy -k bed_mesh`
- CI container: `docker build -f scripts/Dockerfile-build -t dangerklippers/klipper-build:latest .`

### Firmware Build
- `make menuconfig` → `make`
- Artifacts in `out/` (`out/klipper.bin` on ARM, `out/klipper.elf.hex` on AVR)

### Docs
- Build: `cd docs/_kalico && uv run mkdocs build --strict`
- New pages must be added to `docs/_kalico/mkdocs.yml` nav

### Whitespace Check
- `./scripts/check_whitespace.sh`

## Architecture

### Host (Python) — `klippy/`
- Entry: `python -m klippy` → `klippy/printer.py` → `Printer.main()`
- `klippy/extras/`: auto-loaded modules. Config section `[my_module]` maps to `load_config(config)` in `klippy/extras/my_module.py`. Named sections use `load_load_config_prefix(config)`.
- `klippy/plugins/`: user plugins, scanned at startup. Overrides `extras` only when `danger_options.allow_plugin_override` is enabled.
- Module lifecycle, event hooks, object lookup: see `docs/Code_Overview.md`

### MCU Firmware (C) — `src/`
- Architecture-specific: `src/avr/`, `src/stm32/`, etc.
- Extensions: `src/extras/<name>/` needs both `Kconfig` and `Makefile` wiring
- Build system: top-level `Makefile` (builds MCU firmware, not Python)

### Test — `test/`
- pytest suite + `.test` regression collector (`test/klippy/conftest.py`)
- `.test` tests invoke `python -m klippy ...` and require MCU dict files (`--dictdir` / `DICTDIR`)
- `test/klippy_testing/` shims for unit tests without full runtime
- `test/conftest.py` builds `klippy.chelper` — if tests fail early, suspect native build prerequisites

## Debugging Remote Printer (MANDATORY)

When debugging code on the remote Klipper host, **never modify code directly on the host**. Follow this workflow:

1. **Edit locally** — make changes in the local worktree
2. **Commit & push** — push the branch to the remote git repository
3. **SSH to printer** — `ssh klipper@10.42.110.102` (key auth)
4. **Pull on printer** — `cd /home/klipper/klipper && git fetch kalico <branch> && git checkout <branch>`
5. **Restart Klipper** — `curl -X POST http://10.42.110.102/printer/firmware_restart`

**Branch lifecycle**: debug branches are based on `dev`, merged to `test` when complete, then remote debug branch is deleted. Printer runs the `test` branch.

### Config Changes
- Config files can be uploaded directly via Moonraker API: `POST /server/files/upload`
- Restart Klipper after config changes via API or `FIRMWARE_RESTART` gcode

### Printer Info
- Host: `10.42.110.102` | SSH: `klipper@10.42.110.102` (key auth)
- Git remote: `kalico` → `git@github.com:877660224/kalico.git`
- Moonraker API: `http://10.42.110.102`

### Safety
- Z must be above 75mm before moving XY from origin (collision risk)
- Always check position via API before sending movement commands

## Contribution Conventions

- **Commit format**: `module: Capitalized, short summary` (module = file or directory name)
- **Style**: follow surrounding file style; don't enforce generic Python cleanup
- **Doc updates required** for user-facing changes:
  - G-code params → `docs/G-Codes.md`
  - Config params → `docs/Config_Reference.md`
  - Status vars → `docs/Status_Reference.md`
  - API params → `docs/API_Server.md`
  - Breaking changes → `docs/Config_Changes.md`

## Practical Agent Guidance

- Module loading / config parsing / plugin discovery: inspect `klippy/printer.py` first
- Motion / kinematics: read `docs/Code_Overview.md` + `klippy/kinematics/` + `klippy/chelper/`
- Firmware build: verify against `scripts/ci-build.sh`, `scripts/Dockerfile-build`, `test/configs/*.config`
- New modules: add under `klippy/extras/`, load via config sections — follow existing patterns
- New files: include existing copyright-header style

## Personal Dev Workspace
- Config examples: `config/myconfig/` (not committed to mainline `config/`)
- Dev docs: `docs/mydocs/` (committed to fork; update `docs/_kalico/mkdocs.yml` nav when adding)
