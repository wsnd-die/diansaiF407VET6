#include "navigation.h"
#include "bujin.h"
#include <math.h>
#define half_wide_size  8.5f
//旋转PID参数设置定义
float SpeedError0 = 0;     //角度误差
float SpeedError1 = 0;//上次误差
float SpeedErrorInt = 0;//积分误差
float SpeedResult = 0;   //输出结果

float AngleError0 = 0;     //角度误差
float AngleError1 = 0;//上次误差
float AngleErrorInt = 0;
float AngleResult = 0;   //输出结果

float kp_A1 = 0.5; //旋转kp	0.5
float ki_A1 = 0.00;//旋转ki 0
float kd_A1 = 0.5; //旋转kd	0.5
//旋转pid
float kp_A2 = 0.7; //旋转kp	0.5	0.7
float ki_A2 = 0.06;//旋转ki 0	0.06
float kd_A2 = 0.5; //旋转kd	0.5	0.5
float angle_fix = 0;	//偏航角修正

struct move speed = {0, 0, 0};
struct move angle_speed = {0, 0, 0};

float v[2] = {0}; //0为左轮，1为右轮
                  //地址1,4为左轮，2,3为右轮

int TarAngle=0;//目标角度
float TarPos = 360.0;//目标位置

bool is_moving = 0;
/* 圆周率，用于把角度制 yaw 转成弧度制，供 sinf/cosf 使用。 */
#define NAV_PI 3.1415926f

/*
 * 当前导航估计值，单位 cm / 度。
 * 这三个变量用 volatile，是因为它们会在里程计任务更新，在 OLED 显示任务读取。
 */
volatile float g_nav_x_cm = 0.0f;
volatile float g_nav_y_cm = 0.0f;
volatile float g_nav_yaw_deg = 0.0f;

/* 上电初始化时记录陀螺仪的初始 yaw，之后所有 yaw 都减去这个零偏。 */
static float nav_yaw_zero_deg = 0.0f;

/**
  * @brief  把角度限制到 -180~180 度范围内
  * @param  angle 原始角度，单位度
  * @retval 归一化后的角度，单位度
  * @note   例如 190 度会变成 -170 度，-190 度会变成 170 度。
  */
static float Navigation_NormalizeDeg(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }

    while (angle < -180.0f)
    {
        angle += 360.0f;
    }

    return angle;
}

/**
  * @brief  重置导航坐标和 yaw 零点
  * @param  start_x_cm 起点 x 坐标，单位 cm
  * @param  start_y_cm 起点 y 坐标，单位 cm
  * @param  yaw_zero_deg 当前陀螺仪 yaw，作为之后计算的 0 度参考
  * @note   比赛开始时调用一次即可，通常传起始区中心点和当前 HWT101 yaw。
  */
void Navigation_Reset(float start_x_cm, float start_y_cm, float yaw_zero_deg)
{
    g_nav_x_cm = start_x_cm;
    g_nav_y_cm = start_y_cm;
    g_nav_yaw_deg = 0.0f;
    nav_yaw_zero_deg = yaw_zero_deg;
}

/**
  * @brief  根据里程计位移和陀螺仪 yaw 更新当前 x/y 坐标
  * @param  delta_cm 本周期前进/后退距离，单位 cm；正数前进，负数后退
  * @param  yaw_deg  HWT101 当前原始 yaw，单位度
  * @note   delta_cm 为 0 时，不改变 x/y，只刷新当前 yaw。
  */
void Navigation_UpdateByDelta(float delta_cm, float yaw_deg)//delta_cm等于0的时候等价于角度刷新
{
    float yaw_rad;

    /*
     * 坐标约定：
     * 左下角内边线为 (0,0)，场地大小约 520cm x 240cm；
     * yaw 校零后，yaw=0 表示车头朝地图 +Y 方向；
     * delta_cm 为正表示前进，为负表示后退。
     */
    g_nav_yaw_deg = Navigation_NormalizeDeg(yaw_deg - nav_yaw_zero_deg);
    yaw_rad = g_nav_yaw_deg * NAV_PI / 180.0f;

    /*
     * yaw=0 时车头朝 +Y，所以：
     * x 增量 = 位移 * sin(yaw)
     * y 增量 = 位移 * cos(yaw)
     */
    g_nav_x_cm += delta_cm * sinf(yaw_rad);
    g_nav_y_cm += delta_cm * cosf(yaw_rad);
}

/**
  * @brief  获取当前 x 坐标
  * @retval x 坐标，单位 cm
  */
float Navigation_GetXcm(void)
{
    return g_nav_x_cm;
}

/**
  * @brief  获取当前 y 坐标
  * @retval y 坐标，单位 cm
  */
float Navigation_GetYcm(void)
{
    return g_nav_y_cm;
}

/**
  * @brief  获取校零后的当前 yaw
  * @retval yaw 角度，单位度，范围约为 -180~180
  */
float Navigation_GetYawDeg(void)
{
    return g_nav_yaw_deg;
}
void Rotate_Updata(void)
{
	float Angle_Difference;  //定义角度误差
	if(TarAngle>180){
		TarAngle -= 360;
	}else if(TarAngle < -180){
		TarAngle += 360;
	}
	Angle_Difference = TarAngle - g_nav_yaw_deg + angle_fix;
	if(Angle_Difference>180){
		Angle_Difference -=360;
	}else if(Angle_Difference<-180){
		Angle_Difference +=360;
	}
	 //角度死区判断
	if (fabs(Angle_Difference)<0.1) 
	{
		angle_speed.real = 0;
	}
	else
	{
		//累积误差
		AngleError0=AngleError1;
		AngleError1=Angle_Difference;
		//积分分离
		if(fabs(AngleError1)<3)
			AngleErrorInt += AngleError1;
		//PID计算
		AngleResult = -(kp_A2 * AngleError1 + ki_A2 * AngleErrorInt + kd_A2 * (AngleError1 - AngleError0));
		
		//变化死区限制
		if (AngleResult > 30) 
		AngleResult = 30;
		if (AngleResult < -30) 
		AngleResult = -30;
		
		angle_speed.real = AngleResult;
	}
}
void DiffX4_Wheel_Speed_Model_Config(float Velocity, float Palstance)
{ 
	Palstance = -Palstance;

	v[0] = Velocity+Palstance*half_wide_size;
	v[1] = -(Velocity-Palstance*half_wide_size);

	if(v[0] > 0) //左轮正转
	{
		Emm_V5_Vel_Control(left_head, 1, v[0]*10, 255, 1);
		Emm_V5_Vel_Control(left_tail, 1, v[0]*10, 255, 1);
	}
	else         //左轮反转
	{
		Emm_V5_Vel_Control(left_head, 0, (-v[0])*10, 255, 1);
		Emm_V5_Vel_Control(left_tail, 0, (-v[0])*10, 255, 1);
	}

	if(v[1] > 0) //右轮正转
	{
		Emm_V5_Vel_Control(right_head, 1, v[1]*10, 255, 1);
		Emm_V5_Vel_Control(right_tail, 1, v[1]*10, 255, 1);
	}
	else         //右轮反转
	{
		Emm_V5_Vel_Control(right_head, 0, (-v[1])*10, 255, 1);
		Emm_V5_Vel_Control(right_tail, 0, (-v[1])*10, 255, 1);
	}
	Emm_V5_Synchronous_motion(0);
//	Serial5_Printf("\r\nMoveUpdate\r\n");
}
