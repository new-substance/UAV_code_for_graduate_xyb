# 基于啁啾扩频的通感一体化的协议相关技术

**最新设计：峡谷山区、无GNSS的民用监测，已知位置参考终端＋未知位置终端，不使用视觉标志。** 已知终端CSS测距辅助无人机惯性导航，再由无人机定位未知终端；论文主线为通信与测距定位的协议设计。

当前处于模型迁移期。MATLAB代码仍保留旧视觉实现，旧数值配置只供核对，任务仿真尚未放行。旧10终端闭环不构成新场景验证。

## 入口与职责

| 文件 | 用途 |
| --- | --- |
| [论文总纲.md](论文总纲.md) | 唯一研究规格：最新场景、节点职责、协议主线、继承要求与待确认项 |
| [AGENTS.md](AGENTS.md) | 数学审查、协作、版本和输出限额规则 |
| [research_configs/review.json](research_configs/review.json) | 当前设计与放行状态；新模型未审查，mission_enabled=false |
| [research_configs/baseline.json](research_configs/baseline.json) | 唯一配置入口；迁移期保留旧数值，project_context明确其适用范围 |
| [matlab_mvp/README.md](matlab_mvp/README.md) | 唯一MATLAB工程与检查说明 |
| [旧开发结果](evidence/development_cases.json) | 历史逐例正负结果；不验证当前民用无线参考设计 |

`agent.md`与`00_先读我_文件导航.md`仅为兼容链接。旧设计由Git提交fd76b74及其历史保留，不复制第二份生效文档或新建阶段代码目录。

## 只读状态与组件检查

在仓库根目录的MATLAB R2023a会话中执行：

```matlab
addpath('matlab_mvp');
setup_project;
status = project('status'); % 新设计、迁移状态与未放行原因
cfg = project('config');   % 带project_context的旧数值输入
```

`project('check')`仍可核对保留组件的固定输入代数、写入限额及旧4＋1等接口，不启动任务；它尚未包含新已知节点导航和新通感协议的验收。无需为场景文字更新运行整套旧测试。

`project('run', options)`在模型审查完成前拒绝运行。下一步明确山区高程/遮挡、参考布局与误差及帧/业务模型，推导测距辅助惯导与未知节点定位接口，再在此工程迁移并重新审查配置、代码、种子及指标。不得绕过project调用旧全轨迹执行器。

## 版本与结果体积

默认开发50 MiB/批、正式250 MiB/批、单例5 MiB、当期新增1 GiB；保存必要事件和配置/版本清单，不逐例复制源码或全状态轨迹。所有失败和错误接纳保留，旧结果不重记为新模型成功。

源代码、当前规格及小配置由根Git管理。历史原始数据、归档、论文PDF/Word、独立IEEE工具及缓存留在原工作区本地，不随源码库发布；远端不包含旧历史全量复算输入。仅推送当前main，不上传本地历史分支/标签。
