# Unitree G1 sim2real 工作区

本库把 G1-29DoF 的共用底座放在仓库外层，把每项任务的 policy、配置和流程放在独立任务目录。每项任务都要分别完成**仿真**和**真机部署**适配；目前已经接入的任务是 [SuperMimic](supermimic/README.md)。仓库根目录的脚本只负责共用环境、资产校验和按任务编译控制器，并不假定不同任务具有相同的观测、动作或运行流程。

## 目录与职责

| 位置 | 谁使用、负责什么 |
| --- | --- |
| `rl_lab_controller/` | 所有任务共用的 Unitree SDK2/DDS、G1 电机命令路径、FSM/站立与安全基础代码；`g1_29dof_deployment/` 提供按任务编译的 C++ 工程。 |
| `models/` | 共用机器人 XML、网格和 Unitree 提供的模型文档。任务在自身配置中选择模型，避免在任务目录重复存放同一模型。 |
| `simulation_core/` | 共用 MuJoCo 物理配置的加载与校验代码；具体物理数值由任务提供。 |
| `scripts/`、`requirements.txt`、`asset_manifest.json` | 共用 Python/原生库环境、安装与资产检查；清单记录外层共用资产。 |
| `<task>/controller/` | 任务 C++ 入口、策略状态及其接口适配，例如观测构造、动作解释和任务状态转换。 |
| `<task>/config/`、`<task>/requirements.txt` | 任务自己的控制/恢复配置、环境变量和额外 Python 依赖。任务环境配置应先加载 `scripts/env.sh`。 |
| `<task>/assets/` | 任务权重、元数据、动作数据及 `assets/manifest.json`；清单记录任务资产。 |
| `<task>/simulation/`、`<task>/deployment/` | 分别实现任务仿真和真机准备、验证、启动流程；参数与入口由任务自己定义。 |
| `<task>/outputs/` | 任务生成的仿真结果、部署会话与日志；不提交到 Git。 |

**划分原则：**与任务无关、可供其他 G1 任务直接复用的电机通信、模型和基础工具放外层；policy 的输入输出约定、权重、目标动作、站立姿态、增益选择、仿真数值和部署步骤放任务内层。新增任务通过接口使用共用底座，不复制 SDK、整套控制器、模型、`.venv` 或 SuperMimic 的专用实现。确有跨任务需求时，再把经过验证的共性能力加入外层。

`models/README.md`、`models/motor_info.md` 是保留的 Unitree 文档；`models/README_unitree_legacy.md` 保存旧版官方说明。模型目录收录的是本库实际需要的模型子集，并非官方模型全集。

## 环境与资产

当前工作区面向 Ubuntu 24.04 x86_64、Python 3.12。完整工作目录已有大体积资产；新克隆只包含代码、文档、模型描述和少量元数据。SDK 二进制、网格、ONNX 权重及动作 CSV 由单独的资产包提供。所有命令从**仓库根目录**执行，除非明确写了 `cd`：

```bash
# 新克隆时先安装资产包；已有完整工作目录可跳过这一步。
bash scripts/install_asset_bundle.sh ./unitree_sim2real-assets.tar.gz supermimic

# 检查共用资产和任务资产，创建共用 .venv，并为选定任务编译控制器。
bash scripts/setup.sh supermimic
python3 scripts/check_assets.py supermimic
```

`bash scripts/setup.sh <task>` 先安装根目录 `requirements.txt`，再安装 `<task>/requirements.txt`，并用 `-DTASK_ADAPTER_DIR=<task>/controller` 编译共用 C++ 工程。根目录 `.venv/` 由任务共用；`rl_lab_controller/g1_29dof_deployment/build/` 对应**最近一次选定的任务**。切换任务、修改控制器或移动工作目录后，要重新构建并重新准备/验证部署候选，不能沿用旧候选。

资产包与环境变量的细节见 [环境说明](ENVIRONMENT.md) 和 [共用脚本说明](scripts/README.md)。从完整工作目录制作资产包可执行 `bash scripts/make_asset_bundle.sh ./unitree_sim2real-assets.tar.gz supermimic`；资产包不提交 Git。

## 使用现有的 SuperMimic 任务

### 仿真

```bash
bash supermimic/simulation/build.sh
python3 supermimic/simulation/check_inputs.py --policy backpack
.venv/bin/python supermimic/simulation/run_batch.py \
  --policy backpack --name my_batch_001 --workers 3
```

批量入口运行任务内 40 条完整动作；也可选 `--policy baseline` 或 `--policy backpack_retrain`。每次使用新的 `--name`。仿真控制器单独编译，并在只有 `lo` 的隔离网络空间运行；它不会连接真机。SuperMimic 的仿真步长、模型附加参数、FixStand 组合和动作准备规则均由任务目录维护，详见 [仿真说明](supermimic/simulation/README.md)。仿真结果位于 `supermimic/outputs/simulation/<policy>/batches/<name>/`，其中有 `manifest.json`、进度、逐动作结果和日志；运行时加 `--record-video` 可在仿真结束后依次生成逐动作 MP4。

### 真机部署

```bash
cd supermimic
bash deploy.sh check
```

`check` 是离线检查，不查询或控制机器人。`prepare` 和 `run` 必须按 [SuperMimic 真机说明](supermimic/README.md) 在机器人工作站现场执行：先选择 policy、动作和时间窗口，准备候选；默认还会运行隔离仿真验证，也可显式传入 `--skip-simulation` 跳过并留下记录。之后再运行该候选生成的启动命令。脚本会拒绝经 SSH 发起的 `prepare`。部署验证与上面的任务仿真是两个独立流程，不能用一次任务仿真代替部署候选验证。会话与日志集中写入 `supermimic/outputs/deployment/sessions/`。

## 新增任务如何适配

以 `<task>` 表示新的目录名（只使用字母、数字、`_`、`-`）。按下面的顺序接入；`supermimic/` 可作为接口和目录组织的参考，但其中的 policy 协议、动作格式及数值配置需要按新任务重新定义。

1. **定义任务契约。** 明确使用的 `models/` 机器人模型、29 关节顺序、policy 观测与动作格式、控制频率、站立/切换流程、增益和任务安全条件。检查与共用 G1 电机接口的映射；不要靠复制旧权重或旧候选来建立新任务。
2. **建立任务目录与资产清单。** 至少准备 `<task>/controller/main.cpp`、`<task>/controller/src/*.cpp` 和 `<task>/assets/manifest.json`，这是根目录 `setup.sh`/`check_assets.py` 的入口要求。`assets/manifest.json` 使用与 [SuperMimic 清单](supermimic/assets/manifest.json)相同的 `format: 1`、`sha256`、`required_directories`、`bundle_paths` 字段；路径从仓库根目录计算，且不得与外层 `asset_manifest.json` 重复。可在任务内建立 `requirements.txt`、`config/env.sh`、`assets/policies/` 等目录。
3. **接入共用控制器。** 在 `<task>/controller/` 实现任务入口与状态源文件；共用工程通过 `TASK_ADAPTER_DIR` 编入它们。使用外层已有的 DDS、电机命令、FSM 和安全基础设施；任务独有的策略解码与状态逻辑留在内层。运行 `bash scripts/setup.sh <task>`，并核对最终二进制确实属于当前任务。
4. **实现任务仿真。** 在 `<task>/simulation/` 提供模型准备、任务自己的 `physics.json`、仿真入口和结果目录。需要共用物理配置加载器时调用 `simulation_core.physics.apply_profile`。为仿真单独编译带仿真限制的控制器，并限制在 loopback 网络空间；先检查输入，再运行任务自己的完整动作/场景集合，保存配置、哈希、结果和日志。`supermimic/simulation/run_batch.py` 是 SuperMimic 专用入口，不是所有任务通用的运行器。
5. **实现真机部署。** 在 `<task>/deployment/` 和任务入口脚本中提供离线检查、现场候选准备、隔离验证及启动步骤。绑定实际使用的控制器、模型、policy、元数据和任务资产；真机运行前检查候选与验证结果。真机流程要有独立于任务仿真的部署验证机制；若提供跳过选项，必须明确记录该选择并在启动时重新核对候选。不复用仿真专用二进制或自动按键/虚拟支撑逻辑。
6. **写任务 README 并验收。** 说明该任务的资产来源、模型/配置选择、仿真命令、真机现场命令和 `outputs/` 排布。执行 `python3 scripts/check_assets.py <task>`、`bash scripts/setup.sh <task>`，再按任务 README 验证仿真与部署流程。切换回其他任务时重新执行相应任务的 setup 和验证。

共用工程的 [CMake 配置](rl_lab_controller/g1_29dof_deployment/CMakeLists.txt)列出了适配器的编译接口；[共用仿真说明](simulation_core/README.md)解释了物理配置边界。仓库根目录只提供这些共用机制，**新任务的运行命令由其自身 README 定义**。
