#include "arms.h"
#include "bujin.h"

/* 当前升降位置，单位 cm；Move_up/Move_down/Move_Pos 会维护这个值。 */
float now_pos = 0.0f;

/**
  * @brief  升降机构上升指定距离
  * @param  Data_cm 上升距离，单位 cm
  * @note   5 是旧工程使用的升降步进电机地址；
  *         dir=0 表示上升方向；
  *         snF=true 表示先缓存运动命令，随后用同步命令触发。
  */
void Move_up(float Data_cm)
{
    Emm_V5_Pos_Control(5, 0, 200, 200, Data_cm * 10.0f, false, true);
    Emm_V5_Synchronous_motion(0);
    now_pos += Data_cm;
}

/**
  * @brief  升降机构下降指定距离
  * @param  Data_cm 下降距离，单位 cm
  * @note   dir=1 表示下降方向；运动完成后同步更新 now_pos。
  */
void Move_down(float Data_cm)
{
    Emm_V5_Pos_Control(5, 1, 200, 200, Data_cm * 10.0f, false, true);
    Emm_V5_Synchronous_motion(0);
    now_pos -= Data_cm;
}

/**
  * @brief  移动到目标高度位置
  * @param  Tar_pos 目标位置，单位 cm
  * @note   函数会用 Tar_pos - now_pos 算出相对移动距离：
  *         结果为正就上升，结果为负就下降。
  */
void Move_Pos(float Tar_pos)
{
    float move_pos = Tar_pos - now_pos;

    if (move_pos > 0.0f)
    {
        Move_up(move_pos);
    }
    else
    {
        Move_down(-move_pos);
    }

    now_pos = Tar_pos;
}
