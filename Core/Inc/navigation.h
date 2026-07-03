#ifndef __NAVIGATION_H
#define __NAVIGATION_H

#include "main.h"
#include <stdbool.h>

/* 地图坐标：左下角内边线为 (0,0)，x 向右，y 向上，单位 cm。 */
#define NAV_MAP_WIDTH_CM          520.0f
#define NAV_MAP_HEIGHT_CM         240.0f

/* 起始区 500mm x 500mm，中心点约为 (25,25)。 */
#define NAV_START_CENTER_X_CM     25.0f
#define NAV_START_CENTER_Y_CM     25.0f

/* 截图中顶部标注的区域分界线：1000,1000,800,1600,800 mm。 */
#define NAV_A_B_LINE_X_CM         100.0f
#define NAV_B_C_LINE_X_CM         200.0f
#define NAV_C_D_LINE_X_CM         360.0f

/* C 区主要行距 500mm，D 区主要行距 600mm。 */
#define NAV_C_ROW_SPACING_CM      50.0f
#define NAV_D_ROW_SPACING_CM      60.0f
#define NAV_COLUMN_SPACING_CM     70.0f

/* 小车配置 */
#define left_head  4
#define left_tail  2
#define right_head 3
#define right_tail 1
extern volatile float g_nav_x_cm;
extern volatile float g_nav_y_cm;
extern volatile float g_nav_yaw_deg;
extern struct move speed,angle_speed;
extern int TarAngle;
extern float TarPos;
extern bool is_moving;
// extern TickType_t t_move_start;
// extern TickType_t move_delay_ticks;
extern float angle_fix;	//偏航角修正
/* 四轮差速结构体 */
struct move{  
float tar;  //目标速度
float real; //实际速度
float diff; //速度差
};
void Navigation_Reset(float start_x_cm, float start_y_cm, float yaw_zero_deg);
void Navigation_UpdateByDelta(float delta_cm, float yaw_deg);
float Navigation_GetXcm(void);
float Navigation_GetYcm(void);
float Navigation_GetYawDeg(void);
void Rotate_Updata(void);
void DiffX4_Wheel_Speed_Model_Config(float Velocity, float Palstance);

#endif
