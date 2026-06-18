# Kalico Project Guidelines

## Debugging Workflow (MANDATORY)

When debugging code that runs on the remote Klipper host, you MUST follow this workflow. **Never modify code directly on the remote host.**

1. **Edit locally** — Make changes in the local worktree
2. **Commit & push** — Push the branch to the remote git repository
3. **SSH to printer** — Connect to the Klipper host via SSH (`klipper@10.42.110.102`, key auth)
4. **Pull on printer** — `cd /home/klipper/klipper && git fetch kalico <branch> && git checkout <branch>`
5. **Restart Klipper** — Via Moonraker API: `curl -X POST http://10.42.110.102/printer/firmware_restart`

### Printer Info
- Host: `10.42.110.102`
- SSH: `klipper@10.42.110.102` (key auth)
- Git remote on printer: `kalico` → `git@github.com:877660224/kalico.git`
- Moonraker API: `http://10.42.110.102`

### Safety Constraints
- Z must be above 75mm before moving XY from origin (collision risk)
- Always check position via API before sending movement commands
