# MSP mission adapter

本层只负责SW3/SW1语义、车载任务页面、M33状态镜像和任务intent。它不拥有Q2～Q6全局
阶段机，不实现滚球算法，也不直接操作CAN寄存器或电机PWM。

调用方向固定为：`main/runtime -> App/Mission -> mission client -> Drivers/CAN adapter`。
M33是题目阶段的唯一所有者，完整关系见
[`mission-code-layout.md`](../../../../../docs/architecture/mission-code-layout.md)。
