# 单无人机多终端CSS测距定位与服务航点规划

GNSS受限军用演训区临时监测节点定位的研究代码与现行设计文档。项目仅作仿真研究，不代表硬件实测或一般军用环境性能认证。

**当前是可检查的基础源码版本，完整航次尚未放行。** 本轮未增加新仿真；完整多终端闭环尚未验证，现有底座也未达到全域可靠完成。具体研究边界和既有负结果见[论文总纲](论文总纲.md)。

## 研究范围

最终业务目标是：**唤醒发现 → 定位与新位置核验 → 定位信息指导后续服务航线 → 完成其余终端并实际返回**。不要求每个模块各实现一个新算法。

- C1：采用常规唤醒/发现和完整事务，作为工程支撑。
- C2：复用定位、相关质量和4＋1核验底座，不另做新统计理论。
- C3：必要的位置反馈选点功能；先用有限候选单步调整，不强制研究独立的新规划器。

只保留一项待验证候选改进：依据已发现节点的位置估计和服务阶段选择真正有用的下一服务点。当前已有基本位置反馈，是否构成方法增量须通过对照证明；不把“使用了坐标”本身当创新。

这里“用于导航”指后续航线决策，不指RTT更新无人机的位置/姿态。任务期间不用GNSS；未知终端RTT不更新无人机导航；每轮保持4个不同拟合位置＋1个新位置留出核验。标准导航、严格全局统计保证和全局最优规划不扩成新的必做研究。

## 目录与文档职责

| 入口 | 用途 |
| --- | --- |
| [论文总纲.md](论文总纲.md) | 唯一研究规格：场景、约束、实际公式、代码接口和评价口径 |
| [AGENTS.md](AGENTS.md) | 协作、数学审查、版本及输出预算规则 |
| [matlab_mvp/](matlab_mvp/README.md) | 唯一MATLAB主工程，含基础、定位核验与聚合服务三个包 |
| [research_configs/baseline.json](research_configs/baseline.json) | 已展开的冻结V8基础配置，发布仅规范化溯源元数据 |
| [research_configs/review.json](research_configs/review.json) | 范围审查状态；当前不允许新航次 |
| [research_checks/capture_migration_reference.m](research_checks/capture_migration_reference.m) | 独立于历史大数据的固定输入拟合、核验、测距与投影检查 |

`00_先读我_文件导航.md`与`agent.md`仅为兼容链接，不维护第二份规格。名称较早的配置草案只为既有兼容工厂/历史测试保留；新参数入口为`baseline.json`。

## 基础检查

已使用MATLAB R2023a核对以下入口。基础检查使用MATLAB自身功能，不需要原工作区的历史轨迹或源码归档。

```sh
git clone https://github.com/new-substance/UAV_code_for_graduate_xyb.git
cd UAV_code_for_graduate_xyb
```

在仓库根目录启动MATLAB：

```matlab
addpath('matlab_mvp');
setup_project;
project('status');
cfg = project('config');
report = project('check');
assert(report.all_passed && report.mission_count == 0);
```

可选的固定输入核对：

```matlab
addpath('research_checks');
capture_migration_reference('tmp/fixed_input_check.json');
```

这些命令不运行任务航次，也不用于估计任务成功率。`tests/`保留了既有模块测试源码，部分历史用例需要旧环境或涉及任务过程；首版不把“运行全部旧测试”列为独立克隆后的通过标准。

## 新仿真门槛

`project('run')`当前明确拒绝运行；旧的大轨迹写入与全量源码快照入口也已停用。需要先完成采用模型的范围审查、轻量结果写入与字节限额，再考虑新种子评价。参数、独立数据和失败统计规则见总纲及AGENTS。

默认工程限额：开发50 MiB/批、正式250 MiB/批、单例5 MiB、当期新增结果1 GiB。实际写入器尚待完成，这些限额不能被理解为完整仿真已经可用。

## 仓库内容与本地历史

远端首版只包含当前主代码、必要配置、可独立运行的固定输入检查和现行文档。原始仿真数据、压缩轨迹、历史方案/汇报、论文PDF/Word、迁移/存储工具、IEEE独立工具仓库和缓存留在原工作区本地。

本仓库保留已有研究状态与负结果的说明，但不包含全量历史复算输入。远端从精简首版开始；旧本地提交、分支与标签继续本地保留，不自动上传。
