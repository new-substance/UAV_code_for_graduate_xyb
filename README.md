# 单无人机多终端CSS测距定位与服务航点规划

GNSS受限军用演训区临时监测节点定位的研究代码与现行设计文档。项目仅作仿真研究，不代表硬件实测或一般军用环境性能认证。

**已有单终端/5终端历史开发闭环；新名义模型的10终端完整闭环与独立性能待验证。** 用户已确认主线采用物理一致的5 m名义噪声，高度允许候选；当前联合诊断选18 m进行限定开发。实际噪声、精度目标和定位优化不变，不把组件检查当作任务成功。研究边界与负结果见[论文总纲](论文总纲.md)。

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
| [research_configs/baseline.json](research_configs/baseline.json) | 唯一展开配置：5 m名义模型、18 m候选；相对V8三处数值变化见总纲 |
| [research_configs/review.json](research_configs/review.json) | 审查状态、指纹与限定开发规模/种子 |
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
check_bounded_writer();
check_position_feedback();
check_localization_state();
geometry = diagnose_geometry_hypotheses(); % 仅固定输入，不改变基准或运行航次
```

这些命令不运行任务航次，也不用于估计任务成功率。`tests/`保留了既有模块测试源码，部分历史用例需要旧环境或涉及任务过程；首版不把“运行全部旧测试”列为独立克隆后的通过标准。

## 新仿真门槛

`project('run', options)`只接受review.json登记的开发规模/种子；规格、配置或源码变化会使指纹门槛拒绝运行。options包含seeds、count、rule（stage/cost）和唯一label；输出只在results/current/label。旧的大轨迹写入与全量源码快照入口仍停用。先单终端开发，再审查放行5/10终端，独立评价另行冻结。参数、独立数据和失败统计规则见总纲及AGENTS。

默认工程限额：开发50 MiB/批、正式250 MiB/批、单例5 MiB、当期新增结果1 GiB。project('check')验证字节限额、位置反馈、4＋1、名义区域/预测核验和返回预约，不启动航次。最新开发结果、失败与待办统一见[总纲第5—6节](论文总纲.md)，不在各README重复维护计数。

本地结果只用一个自动汇总入口，逐例审计后生成results/current_overview.json：

```powershell
python -X utf8 research_checks/summarize_managed_results.py
```

[开发结果快照](evidence/development_cases.json)保留各版本逐例正负结果，[固定输入几何诊断](evidence/geometry_hypotheses.json)保留候选高度的名义检查；两者均不构成独立成功率或完整物理可行性证明。

## 仓库内容与本地历史

远端首版只包含当前主代码、必要配置、可独立运行的固定输入检查和现行文档。原始仿真数据、压缩轨迹、历史方案/汇报、论文PDF/Word、迁移/存储工具、IEEE独立工具仓库和缓存留在原工作区本地。

本仓库保留已有研究状态与负结果的说明，但不包含全量历史复算输入。远端从精简首版开始；旧本地提交、分支与标签继续本地保留，不自动上传。
