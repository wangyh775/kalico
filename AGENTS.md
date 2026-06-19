# AGENTS.md

## 适用范围
- 适用于整个仓库。

## 项目简介
- Kalico 是 Klipper 的社区维护分支。谨慎对待上游 Klipper 的假设；本仓库在主线 Klipper 之上添加了行为和模块（`README.md`、`docs/Kalico_Additions.md`）。
- 仓库同时包含主机端 Python 代码和 MCU 固件代码。许多修改需要同时检查两端。

## 重要目录
- `klippy/`：主机固件 Python 运行时。
  - 入口为 `python -m klippy` / `klippy/klippy.py`，分发到 `klippy.printer.main()`。
  - `klippy/extras/` 包含自动加载的主机模块。
  - `klippy/plugins/` 在启动时也会被扫描；插件名可以覆盖 `extras`，但仅当配置中启用 `danger_options.allow_plugin_override` 时（`klippy/printer.py`）。
- `src/`：MCU 固件 C 源码。架构特定代码位于子目录如 `src/avr`、`src/stm32` 等。
- `test/`：基于 pytest 的测试套件加上自定义 `.test` 回归收集器用于 Klippy 集成测试。
- `test/configs/`：CI 中用于固件编译覆盖的标准 MCU `.config` 配置文件。
- `docs/`：用户/开发文档。`docs/Code_Overview.md`、`docs/Debugging.md` 和 `docs/CONTRIBUTING.md` 是架构和贡献期望的主要参考。
- `docs/_kalico/`：文档站点的 MkDocs 项目。
- `scripts/`：构建/测试辅助工具、回归运行器、CI 容器资源、调试工具。

## 开发环境和核心命令
- Python 基线版本为 `>=3.9`（`pyproject.toml`）。
- 使用 uv 安装开发依赖，不要用 ad-hoc pip：
  - `uv sync --dev`
- Ruff 是本仓库唯一配置的格式化器/检查器：
  - `uv run ruff check .`
  - `uv run ruff format .`
- Pre-commit 使用 `--fix` 和 `ruff-format` 运行 Ruff，排除 `docs/`、`config/` 和 `lib/`（`.pre-commit-config.yaml`）。
  - `uv run pre-commit run --all-files`

## 测试和验证工作流
- 快速主机端 pytest：
  - `uv run pytest`
- CI 在 Docker 构建镜像中跨 Python 3.9 到 3.14 运行 pytest，通常使用 xdist（`.github/workflows/ci-build_test.yaml`）。
- 自定义 `.test` 回归套件由 `test/klippy/conftest.py` 收集。这些测试调用 `python -m klippy ...` 并需要 MCU 字典文件（`--dictdir` / `DICTDIR`）。
- `test/conftest.py` 会急切构建/加载 `klippy.chelper`（通过 `klippy.chelper.get_ffi()`）。如果测试非常早期失败，优先怀疑 chelper/原生构建前置条件。
- 常用的聚焦测试调用：
  - 单个 pytest 文件：`uv run pytest test/test_autosave.py`
  - 单个 pytest 测试：`uv run pytest test/test_autosave.py::test_autosave_includes`
  - Klippy `.test` 回归子集（需要字典）：`uv run pytest test/klippy -k bed_mesh`
- 完整的 CI 风格本地验证由容器驱动：
  - `docker build -f scripts/Dockerfile-build -t dangerklippers/klipper-build:latest .`
  - 然后，例如：`docker run -v ${PWD}:/klipper dangerklippers/klipper-build:latest --python 3.12 py.test -n auto`
- `scripts/ci-build.sh` 是固件+pytest CI 行为的可执行参考：
  - 编译所有 `test/configs/*.config`
  - 将生成的 `out/klipper.dict` 文件复制到 `DICTDIR`
  - 然后运行 `py.test`

## 固件构建注意事项
- 顶层 `Makefile` 构建 MCU 固件，不是主机 Python 包。
- 常用固件流程：
  - `make menuconfig`
  - `make`
- 构建产物位于 `out/`。`docs/Code_Overview.md` 记录了最终输出如 ARM 平台的 `out/klipper.bin` 或 AVR 平台的 `out/klipper.elf.hex`。
- 构建会自动包含 `src/extras/Makefile`。`src/extras/<name>/` 下的固件扩展需要同时配置 `Kconfig` 和 `Makefile` 才能参与构建/menuconfig（`docs/Code_Overview.md`）。

## 主机模块规范
- 新主机模块通常应添加到 `klippy/extras/` 下，并通过配置节加载。
- Kalico 按节名自动加载模块：
  - `[my_module]` → `klippy/extras/my_module.py` 中的 `load_config(config)`
  - `[my_module name]` → `load_config_prefix(config)`
- 此规范在 `klippy/extras/` 中广泛使用；遵循现有模块而不是发明替代注册模式。
- `docs/Code_Overview.md` 是模块生命周期、事件钩子和对象查找规范的最佳本地指南。在添加或重构 extras 之前请先阅读。

## 测试注意事项
- `test/conftest.py` 创建从 `test/klippy_testing_plugin.py` 到 `klippy/plugins/testing.py` 的符号链接。不要破坏此插件加载路径。
- `test/klippy_testing/` 垫片用于无完整运行时的单元式测试。
- 旧式 `scripts/test_klippy.py` 存在但 `test/klippy/conftest.py` 下的 pytest 是当前机制。

## 文档工作流
- 文档从 `docs/` 构建，使用 `docs/_kalico/` 中的 MkDocs 项目。
- 严格文档构建命令：
  - `cd docs/_kalico && uv run mkdocs build --strict`
- 如果添加新文档页面，还须更新 `docs/_kalico/mkdocs.yml` 的 nav；`docs/CONTRIBUTING.md` 明确指出这一点。
- **数学公式格式**：
  - 块公式（单独显示在一行）：使用 `$$ ... $$`
    - 示例：`$$ \mathbf{x} = \begin{bmatrix} T_h \\ T_b \\ T_s \end{bmatrix} $$`
  - 行内公式（嵌入文本中）：使用 `$ ... $`
    - 示例：`变量 $x$ 表示温度`
  - MathJax 3 在 `docs/_kalico/javascripts/mathjax.js` 中配置，使用 pymdownx.arithmatex 扩展

## 面向用户变更的文档更新要求
- `docs/CONTRIBUTING.md` 明确要求：面向用户的代码变更必须更新参考文档。
- 至少在相关时更新对应的文档源：
  - G-code / 命令参数 → `docs/G-Codes.md`
  - 配置模块 / 参数 → `docs/Config_Reference.md`
  - 状态变量 → `docs/Status_Reference.md`
  - Webhooks / API 参数 → `docs/API_Server.md`
  - 破坏性配置/命令变更 → `docs/Config_Changes.md`

## 代码风格和贡献约束
- 遵循周围文件风格，而不是强制通用现代 Python 清理。项目明确偏好与现有代码流/格式的一致性（`docs/CONTRIBUTING.md`）。
- 避免将仅空白编辑与功能性更改混合。
- 修复根本原因；贡献指导明确期望缺陷修复针对根本原因。
- 新的 Python/C 源文件应包含现有的版权声明头样式。

## 提交 / PR 期望
- 提交主题格式为 `module: 首字母大写的简短描述`，其中 `module` 通常是仓库文件或目录名（`docs/CONTRIBUTING.md`）。
- 提交应是单一主题且独立合理。
- 项目贡献政策要求 Signed-off-by 行。
- **提交前运行 pre-commit 检查**：
  - 安装钩子一次：`uv run pre-commit install`
  - 钩子在 `git commit` 时自动运行，或手动运行：`uv run pre-commit run --all-files`
  - Pre-commit 使用 `--fix` 和 `ruff-format` 运行 Ruff（排除 `docs/`、`config/`、`lib/`）
- **运行空白检查**（源代码变更时）：
  - `./scripts/check_whitespace.sh`
  - 检查 C/H/Python/Shell/Markdown 文件中的尾随空白、制表符与空格问题
  - 空白-only 变更不应与功能性更改混合
- **运行文档构建检查**（如果修改了文档）：
  - `cd docs/_kalico && uv run mkdocs build --strict`
  - 严格模式对任何警告（断开的链接、缺失的 nav 条目等）都会失败
  - `docs/` 中 `.md` 文件的所有变更都需要此检查

## 调试命令
- 空白检查：`./scripts/check_whitespace.sh`
- 运行 Klippy 进行调试：`python ./klippy/klippy.py ~/printer.cfg -i test.gcode -o test.serial -v -d out/klipper.dict`
- 解析串口输出：`python ./klippy/parsedump.py out/klipper.dict test.serial > test.txt`

## 实用代理指导
- 修改模块加载、配置解析、插件发现或打印机启动流程时，先检查 `klippy/printer.py`。
- 修改运动/运动学行为时，先阅读 `docs/Code_Overview.md` 和 `klippy/kinematics/` 及 `klippy/chelper/` 中的相关文件。
- 修改固件构建行为时，对照 `scripts/ci-build.sh`、`scripts/Dockerfile-build` 和 `test/configs/*.config` 验证，而不是从 README 文本猜测。

## 个人开发工作区
- **个人配置示例**：存储在 `config/myconfig/` 目录。
  - 示例：`config/myconfig/alps_serial_example.cfg`
  - 这些是个人测试配置，不提交到主线 `config/` 目录。
- **个人开发文档**：存储在 `docs/mydocs/` 目录。
  - 示例：`docs/mydocs/MPC v2 算法技术文档.md`
  - 这些是个人笔记/实验，可以包含在你的个人文档站点中。
  - **重要**：添加新文档到 `docs/mydocs/` 时，还必须更新 `docs/_kalico/mkdocs.yml` 导航：
    - 将新文档添加到 `nav` 配置中的"我的文档"部分
    - 可以组织为子类别（如 `MPCV2:`）以获得更好的结构
    - 示例结构：
      ```yaml
      - 我的文档:
        - MPCV2:
          - mydocs/MPCV2/MPC v2 算法技术文档.md
          - mydocs/MPCV2/MPC_V2.md
        - mydocs/串口压力传感器.md
      ```
  - **Git 跟踪**：与 `config/myconfig/` 不同，`docs/mydocs/` 可以提交并推送到你的分支用于个人文档站点部署。
