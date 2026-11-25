# 唯一MATLAB主工程

规格见[论文总纲](../论文总纲.md)，来源见[文献与原理依据](../文献与原理依据.md)，执行规则见[AGENTS](../AGENTS.md)。当前采用GNSS不可用林区、已知参考CSS辅助IMU、未知节点通信与定位的模型，不使用视觉标志。

## 配置和固定输入检查

```matlab
addpath('matlab_mvp');
setup_project;
project('status');
cfg = project('config'); % 新模型白名单，不含构造目标、fixed_review或旧视觉输入
result = project('check'); % 新组件固定输入回归，不启动任务或世界积分
```

load_project_config('legacy')仅用于历史组件兼容；旧工厂/视觉代码和旧结果保留历史身份。project('check',struct('legacy',true))是原组件检查入口，不能据其通过证明新模型。默认路径只加入本工程根，不使用仓库genpath。

## 已迁移接口

| 同一+uavmvp包中的模块 | 职责 |
| --- | --- |
| forest_frame / forest_radio | 7类网络序应用帧、有符号RAW与两方向配对、PHY费用、监听保护和分角色链路模型 |
| forest_navigation / forest_covariance | 静置联合初始化、IMU传播、已知参考更新、有限相关天线副本、名义导航门槛 |
| forest_localization | 有限网格MAP/规划区域、完整4组质量、新第五点相关条件混合及完整块核验 |
| forest_protocol | 身份/发现、数据分片、数据与Q满足后申请第五点、一次判决、停测/通知ACK、有限票据与共享重试 |
| forest_policy | 有限覆盖图、完整服务账本准入、同能力B0/B1动作选择和路径首点；全窗口预测/资格由调用方提供 |
| forest_motion | 估计反馈命令、独立推力/接触数学接口、持续CSS下降准入及离线真实误差评价 |

验收入口为research_checks/check_forest_components.m。保存输入无世界对象、随机航次、运动积分或结果批次；包含负值RAW、历史相关修正、错误角色拒绝、留出重用拒绝、反向残差、数据去重、有限重试、期限/导航拒绝和实际着陆与在线完成分离等检查。

## 当前边界

数学/状态接口通过不等于整航次已经集成或可行。候选资格与全窗口预测提供器、实际包/IMU时轴、环境积分和任务/轻量写入适配仍需集成审查。LOC默认检查计算超时；固定输入回归显式使用fixed_input_review复核数值，不证明50 ms预约兑现，也不能转为任务调用模式。

project('run',options)当前在创建任务与输出前拒绝。即使误改放行标志，也不会回落到旧视觉执行器。用户“不仿真”要求和mission_enabled=false持续有效；源码版本、失败留存及字节限额继续按AGENTS执行。
