# H-ball mission profiles

这里是Q1服务和Q2～Q6评分任务参数的唯一人工编辑源。每题使用独立YAML文件，内容包括
deadline、阶段阈值、底盘profile、滚球目标/稳定窗、视觉门限、录像策略和版本。后续生成器
为MSP、M33、M55和Pi产生只读结构及同一`config_hash`；生成文件不得手工修改。

在profile schema和生成器的host测试完成前，不创建伪造的正式参数，也不把新增题目常量
散落到各端源码。目录和调用规则见
[`mission-code-layout.md`](../../docs/architecture/mission-code-layout.md)。
