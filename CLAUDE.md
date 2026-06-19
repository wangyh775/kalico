# CLAUDE.md

本文件为 Claude Code (claude.ai/code) 在本仓库中工作时提供指导。

## 项目概述

Kalico 是 Klipper 3D 打印机固件的社区维护分支。它在主线 Klipper 基础上添加了额外功能和行为（详见 `docs/Kalico_Additions.md`）。仓库同时包含主机端 Python 代码和 MCU 固件 C 代码——许多修改需要同时检查两端。

## 开发命令

### Python 环境
- Python >= 3.9（`pyproject.toml`）
- 安装依赖：`uv sync --dev`

### 代码检查与格式化
- `uv run ruff check .` — 代码检查
- `uv run ruff format .` — 代码格式化
- 预提交钩子：`uv run pre-commit run --all-files`

### 测试
- 主机端 pytest：`uv run pytest`
- 单个文件：`uv run pytest test/test_autosave.py`
- 单个测试：`uv run pytest test/test_autosave.py::test_autosave_includes`
- Klippy 回归测试（需要 MCU 字典文件）：`uv run pytest test/klippy -k bed_mesh`
- CI 容器：`docker build -f scripts/Dockerfile-build -t dangerklippers/klipper-build:latest .`

### 固件构建
- `make menuconfig` → `make`
- 构建产物在 `out/` 目录（ARM 平台为 `out/klipper.bin`，AVR 平台为 `out/klipper.elf.hex`）

### 文档
- 构建：`cd docs/_kalico && uv run mkdocs build --strict`
- 新页面必须添加到 `docs/_kalico/mkdocs.yml` 的 nav 中

### 空白检查
- `./scripts/check_whitespace.sh`

## 架构

### 主机端（Python）— `klippy/`
- 入口：`python -m klippy` → `klippy/printer.py` → `Printer.main()`
- `klippy/extras/`：自动加载模块。配置节 `[my_module]` 映射到 `klippy/extras/my_module.py` 中的 `load_config(config)`。命名节使用 `load_config_prefix(config)`。
- `klippy/plugins/`：用户插件，启动时扫描。仅当 `danger_options.allow_plugin_override` 启用时才覆盖 `extras`。
- 模块生命周期、事件钩子、对象查找：参见 `docs/Code_Overview.md`

### MCU 固件（C）— `src/`
- 架构特定代码：`src/avr/`、`src/stm32/` 等
- 扩展：`src/extras/<name>/` 需要同时配置 `Kconfig` 和 `Makefile`
- 构建系统：顶层 `Makefile`（构建 MCU 固件，非 Python）

### 测试 — `test/`
- pytest 测试套件 + `.test` 回归收集器（`test/klippy/conftest.py`）
- `.test` 测试调用 `python -m klippy ...` 并需要 MCU 字典文件（`--dictdir` / `DICTDIR`）
- `test/klippy_testing/` 用于无完整运行时的单元测试垫片
- `test/conftest.py` 构建 `klippy.chelper`——如果测试早期失败，优先检查原生构建前置条件

## 远程打印机调试（强制要求）

调试远程 Klipper 主机上的代码时，**绝不能直接修改远程主机代码**。必须遵循以下工作流：

1. **本地编辑** — 在本地工作树中修改
2. **提交并推送** — 将分支推送到远程 Git 仓库
3. **SSH 连接打印机** — `ssh klipper@10.42.110.102`（密钥认证）
4. **打印机拉取** — `cd /home/klipper/klipper && git fetch kalico <branch> && git checkout <branch>`
5. **重启 Klipper** — `curl -X POST http://10.42.110.102/printer/firmware_restart`

**分支生命周期**：调试分支基于最新的 `test` 远程分支创建工作树，创建工作树前应先更新本地`test`分支，开发完成后经用户允许创建PR合并到 `test`。打印机运行 `test` 分支。

### 配置修改
- 配置文件可通过 Moonraker API 直接上传：`POST /server/files/upload`
- 配置修改后需重启 Klipper（API 或 `FIRMWARE_RESTART` G-code）

### 打印机信息
- 主机：`10.42.110.102` | SSH：`klipper@10.42.110.102`（密钥认证）
- Git 远程：`kalico` → `git@github.com:877660224/kalico.git`
- Moonraker API：`http://10.42.110.102`

### 安全
- Z 轴必须高于 75mm 才能移动 XY 轴离开原位（碰撞风险）
- 发送移动命令前必须通过 API 检查位置

## 贡献规范

- **提交格式**：`module: 首字母大写的简短描述`（module = 文件或目录名）
- **代码风格**：遵循周围文件风格，不要强制通用 Python 清理
- **面向用户的修改必须更新文档**：
  - G-code 参数 → `docs/G-Codes.md`
  - 配置参数 → `docs/Config_Reference.md`
  - 状态变量 → `docs/Status_Reference.md`
  - API 参数 → `docs/API_Server.md`
  - 破坏性变更 → `docs/Config_Changes.md`

## 实用代理指导

- 模块加载/配置解析/插件发现：先检查 `klippy/printer.py`
- 运动/运动学：先阅读 `docs/Code_Overview.md` + `klippy/kinematics/` + `klippy/chelper/`
- 固件构建：对照 `scripts/ci-build.sh`、`scripts/Dockerfile-build`、`test/configs/*.config` 验证
- 新模块：添加到 `klippy/extras/`，通过配置节加载——遵循现有模式
- 新文件：包含现有的版权声明头样式

## 个人开发工作区
- 配置示例：`config/myconfig/`（不提交到主线 `config/`）
- 开发文档：`docs/mydocs/`（提交到分支；添加时更新 `docs/_kalico/mkdocs.yml` 的 nav）
