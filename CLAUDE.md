# Kalico Project Guidelines

## Debugging Workflow (MANDATORY)

When debugging code that runs on the remote Klipper host, you MUST follow this workflow. **Never modify code directly on the remote host.**

1. **Edit locally** — Make changes in the local worktree
2. **Commit & push** — Push the branch to the remote git repository
3. **SSH to printer** — Connect to the Klipper host via SSH (`klipper@10.42.110.102`, key auth)
4. **Pull on printer** — `cd /home/klipper/klipper && git fetch kalico <branch> && git checkout <branch>`
5. **Restart Klipper** — Via Moonraker API: `curl -X POST http://10.42.110.102/printer/firmware_restart`

## Config Changes

- Config files (printer.cfg, RP2040.cfg, etc.) can be edited directly via Moonraker file API
- Use `POST /server/files/upload` to upload modified config files
- Restart Klipper after config changes via API or `FIRMWARE_RESTART` gcode

## Printer Info
- Host: `10.42.110.102`
- SSH: `klipper@10.42.110.102` (key auth, user: klipper)
- Git remote on printer: `kalico` → `git@github.com:877660224/kalico.git`
- Moonraker API: `http://10.42.110.102`

## Safety Constraints
- Z must be above 75mm before moving XY from origin (collision risk)
- Always check position via API before sending movement commands

## Code Style & Contribution
- Follow surrounding file style; don't enforce generic Python cleanup
- Commit subject format: `module: Capitalized, short summary`
- User-facing changes must update docs (G-Codes.md, Config_Reference.md, etc.)
- Run `uv run ruff check .` and `uv run ruff format .` before committing
- Whitespace check: `./scripts/check_whitespace.sh`

## Project Structure
- `klippy/`: host firmware Python runtime
- `klippy/extras/`: auto-loaded host modules (config section → `load_config()`)
- `src/`: MCU firmware C sources
- `test/`: pytest suite + `.test` regression tests
- `docs/`: user/dev docs (MkDocs project in `docs/_kalico/`)
