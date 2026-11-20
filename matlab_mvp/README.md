# 唯一MATLAB主工程

研究规格与公式只看根目录[论文总纲](../论文总纲.md)，执行规则看[AGENTS.md](../AGENTS.md)。

```matlab
addpath('matlab_mvp');
setup_project;
project('status'); % 只读状态，含当前审查限制
cfg = project('config'); % 已展开的冻结V8基础配置
project('check'); % 小型确定性检查，不运行航次
```

主工程内保留+uavmvp基础、+uavstage定位/核验、+uavpriority聚合/服务包，包名暂保留以避免改变数值实现；不再跨两个阶段目录加载源码。setup_stage/setup_priority仅为兼容别名。

物理数值和定位后端保持不变，旧V8非线性似然仍是实际实现。新增ResultWriter和choose_service_point分别完成有界写入与位置/阶段反馈提案的组件检查；后者仍须实际导航/返回准入。execute_task接入共享一次飞行的多终端协调，性能仍待验证；不为C1/C2/C3标签分别新建算法。

project('run', options)按review.json及规格/配置/源码指纹限定开发运行，首轮仅单终端；不自动放行独立评价。project('check')含确定性的写入限额及候选提案检查，任务数为0。旧运行/全量快照入口已停用，不能直接绕过执行。不要运行全部旧测试来替代本次小型检查，其中含任务过程。

历史执行器、计划和正式结果仅保留在原工作区本地，未随本仓库发布，也不是基础检查的依赖。版本由根Git提交标识，不再每例复制源码。tests/保留既有模块测试源码，部分历史用例依赖旧环境；首版可独立运行的检查以根README列出的命令为准。

3次单终端开发尝试已保留，其中首次接线错误已修复；修正后两例均未通过在线质量判定。当前review.json要求先审查保存输入中的质量规则，关闭新航次；组件检查仍可运行。
