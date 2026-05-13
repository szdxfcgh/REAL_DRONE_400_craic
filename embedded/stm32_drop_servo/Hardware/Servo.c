#include "stm32f10x.h"
#include "PWM.h"

void Servo_Init(void)
{
    PWM_Init();
}

static uint16_t Servo_AngleToCompare(float Angle)
{
    if (Angle < 0)
    {
        Angle = 0;
    }
    else if (Angle > 180)
    {
        Angle = 180;
    }

    return (uint16_t)(Angle / 180 * 2000 + 500);
}

void Servo_SetAngle(uint8_t ServoId, float Angle)
{
    uint16_t Compare;

    Compare = Servo_AngleToCompare(Angle);
    if (ServoId == 1)
    {
        PWM_SetCompare2(Compare);
    }
    else if (ServoId == 2)
    {
        PWM_SetCompare3(Compare);
    }
    else if (ServoId == 3)
    {
        PWM_SetCompare4(Compare);
    }
}
