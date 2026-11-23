# 唯一MATLAB主工程

最新规格见根目录[论文总纲](../论文总纲.md)，执行规则见[AGENTS](../AGENTS.md)。当前主题为无GNSS峡谷山区的民用CSS通感协议，采用已知/未知两类终端，不使用视觉标志；已知终端CSS测距辅助无人机惯性导航，再定位未知终端。

**迁移尚未完成，任务未放行。** +uavmvp、+uavstage、+uavpriority保留旧数值实现；camera_model、nav_update_reference及旧视觉返回/候选路径仍是历史代码，不因更新题目而成为无线参考导航。旧结果保持不变。

```matlab
addpath('matlab_mvp');
setup_project;
project('status'); % 当前设计、旧实现状态及未放行原因
cfg = project('config'); % project_context标明旧数值输入的范围
```

project('check')仅针对现有保留组件的固定输入/限额检查，任务数为0；尚不验证新协议、已知节点辅助导航或新场景性能。project('run', options)先检查review.json，当前在创建任务和输出前拒绝运行。

后续继续使用本工程迁移测距、两类节点协议、导航输入与返回模型；不另建阶段工程，不直接将视觉标志坐标改名为无线参考。按总纲先推导和审查，再实现与限定仿真。源码由Git标识，结果采用字节限额，不复制每例源码。
