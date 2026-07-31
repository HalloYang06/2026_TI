# MSP mission adapter

本层只负责SW3/SW1语义、车载任务页面、M33状态镜像和任务intent。它不拥有Q2～Q6全局
阶段机，不实现滚球算法，也不直接操作CAN寄存器或电机PWM。

调用方向固定为：`main/runtime -> App/Mission -> mission client -> Drivers/CAN adapter`。
M33是题目阶段的唯一所有者，完整关系见
[`mission-code-layout.md`](../../../../../docs/architecture/mission-code-layout.md)。

`hball_mission_run_guard`只决定MSP底盘是否仍有运行许可。Q2本地运行不依赖M33状态；
Q4～Q6必须持续看到相同任务和epoch的新鲜`RUNNING`状态，并在M33完成、中止或状态失联时
请求本地停车。它不推进M33阶段，也不调用CAN、显示或执行器。
