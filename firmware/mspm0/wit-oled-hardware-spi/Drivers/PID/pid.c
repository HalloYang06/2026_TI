#include "ti_msp_dl_config.h"             // Device header
#include "pid.h"
#include <math.h>

/*定义PID结构体变量*/
PID_t LEFT = {					//内环PID结构体变量，定义的时候同时给部分成员赋初值
	.Kp = 0,					//比例项权重
	.Ki = 0,					//积分项权重
	.Kd = 0,					//微分项权重
	.OutMax = 100,				//输出限幅的最大值
	.OutMin = -100,				//输出限幅的最小值
};

PID_t RIGHT = {					//外环PID结构体变量，定义的时候同时给部分成员赋初值
	.Kp = 0,					//比例项权重
	.Ki = 0,					//积分项权重
	.Kd = 0,					//微分项权重
	.OutMax = 100,				//输出限幅的最大值
	.OutMin = -100,				//输出限幅的最小值
};
PID_t ANGLE={
	.Kp = 0,
	.Ki = 0,
	.Kd = 0,
	.OutMax = 100,
	.OutMin = -100,
};

float angle_deadband = 1.0f;  // 角度死区阈值，可根据实际需求调整



void PID_Update(PID_t *p)
{
    /*获取本次误差和上次误差*/
    p->Error1 = p->Error0;                  //获取上次误差

    p->Error0 = p->Target - p->Actual;      //获取本次误差，目标值减实际值，即为误差值
    
    /* 角度PID死区处理 */
    // 判断当前是否是角度PID
    if (p == &ANGLE)
    {
        /* WIT 航向角范围为 [-180, 180]，始终选择最短旋转方向。 */
        while (p->Error0 > 180.0f) {
            p->Error0 -= 360.0f;
        }
        while (p->Error0 < -180.0f) {
            p->Error0 += 360.0f;
        }

        // 如果误差绝对值小于死区阈值，则将误差设为0
        if (fabsf(p->Error0) < angle_deadband)
        {
            p->Error0 = 0.0f;
        }
    }
    
    /*外环误差积分（累加）*/
    /*如果Ki不为0，才进行误差积分，这样做的目的是便于调试*/
    /*因为在调试时，我们可能先把Ki设置为0，这时积分项无作用，误差消除不了，误差积分会积累到很大的值*/
    /*后续一旦Ki不为0，那么因为误差积分已经积累到很大的值了，这就导致积分项疯狂输出，不利于调试*/
    if (p->Ki != 0)                    //如果Ki不为0
    {
        p->ErrorInt += p->Error0;      //进行误差积分
    }
    else                                //否则
    {
        p->ErrorInt = 0;               //误差积分直接归0
    }
    
    /*PID计算*/
    /*使用位置式PID公式，计算得到输出值*/
    p->Out = p->Kp * p->Error0
           + p->Ki * p->ErrorInt
           + p->Kd * (p->Error0 - p->Error1);
    
    /*输出限幅*/
    if (p->Out > p->OutMax) {p->Out = p->OutMax;}    //限制输出值最大为结构体指定的OutMax
    if (p->Out < p->OutMin) {p->Out = p->OutMin;}    //限制输出值最小为结构体指定的OutMin

}

/**
  * 函    数：PID计算及结构体变量值更新
  * 参    数：PID_t * 指定结构体的地址
  * 返 回 值：无
  */
// void PID_Update(PID_t *p)
// {
// 	/*获取本次误差和上次误差*/

// 	p->Error1 = p->Error0;					//获取上次误差

// 	p->Error0 = p->Target - p->Actual;		//获取本次误差，目标值减实际值，即为误差值
	
// 	/*外环误差积分（累加）*/
// 	/*如果Ki不为0，才进行误差积分，这样做的目的是便于调试*/
// 	/*因为在调试时，我们可能先把Ki设置为0，这时积分项无作用，误差消除不了，误差积分会积累到很大的值*/
// 	/*后续一旦Ki不为0，那么因为误差积分已经积累到很大的值了，这就导致积分项疯狂输出，不利于调试*/
// 	if (p->Ki != 0)					//如果Ki不为0
// 	{
// 		p->ErrorInt += p->Error0;	//进行误差积分
// 	}
// 	else							//否则
// 	{
// 		p->ErrorInt = 0;			//误差积分直接归0
// 	}
	
// 	/*PID计算*/
// 	/*使用位置式PID公式，计算得到输出值*/
// 	p->Out = p->Kp * p->Error0
// 		   + p->Ki * p->ErrorInt
// 		   + p->Kd * (p->Error0 - p->Error1);
	
// 	/*输出限幅*/
// 	if (p->Out > p->OutMax) {p->Out = p->OutMax;}	//限制输出值最大为结构体指定的OutMax
// 	if (p->Out < p->OutMin) {p->Out = p->OutMin;}	//限制输出值最小为结构体指定的OutMin

// }
