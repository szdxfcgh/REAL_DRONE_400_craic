#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Servo.h"
#include "Key.h"
#include <string.h>

#define SERVO_LOCK_ANGLE        0
#define SERVO_DROP1_ANGLE       45
#define SERVO_DROP2_ANGLE       90
#define SERVO_DROP3_ANGLE       135

#define SERVO_ACTION_DELAY_MS   700
#define SERIAL_RX_BUF_SIZE      32

uint8_t KeyNum;
float Angle = SERVO_LOCK_ANGLE;

static char Serial_RxBuffer[SERIAL_RX_BUF_SIZE];
static uint8_t Serial_RxIndex;
static uint8_t ServoBusy;

static void Serial_Init(void);
static void Serial_SendChar(char Char);
static void Serial_SendString(const char *String);
static uint8_t Serial_IsServoCommand(const char *Command);
static void Serial_ProcessRx(void);
static void Serial_HandleCommand(const char *Command);
static void Servo_DoAction(float TargetAngle, const char *AckString);
static void Servo_ActionDelay(uint16_t DelayMs);

static void Serial_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    /* USART1_TX: PA9 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART1_RX: PA10 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);

    USART_Cmd(USART1, ENABLE);
}

static void Serial_SendChar(char Char)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, (uint16_t)Char);
}

static void Serial_SendString(const char *String)
{
    while (*String != '\0')
    {
        Serial_SendChar(*String);
        String++;
    }
    Serial_SendChar('\r');
    Serial_SendChar('\n');
}

static uint8_t Serial_IsServoCommand(const char *Command)
{
    if (strcmp(Command, "DROP:1") == 0) { return 1; }
    if (strcmp(Command, "DROP:2") == 0) { return 1; }
    if (strcmp(Command, "DROP:3") == 0) { return 1; }
    if (strcmp(Command, "LOCK") == 0) { return 1; }
    return 0;
}

static void Serial_ProcessRx(void)
{
    char Command[SERIAL_RX_BUF_SIZE];
    uint8_t Data;

    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
    {
        Data = (uint8_t)USART_ReceiveData(USART1);

        if (Data == '\n' || Data == '\r')
        {
            if (Serial_RxIndex > 0)
            {
                Serial_RxBuffer[Serial_RxIndex] = '\0';
                strcpy(Command, Serial_RxBuffer);
                Serial_RxIndex = 0;
                Serial_HandleCommand(Command);
            }
        }
        else
        {
            if (Serial_RxIndex < SERIAL_RX_BUF_SIZE - 1)
            {
                Serial_RxBuffer[Serial_RxIndex] = (char)Data;
                Serial_RxIndex++;
            }
            else
            {
                Serial_RxIndex = 0;
                Serial_SendString("ERR:INVALID");
            }
        }
    }
}

static void Serial_HandleCommand(const char *Command)
{
    if (ServoBusy != 0 && Serial_IsServoCommand(Command) != 0)
    {
        Serial_SendString("ERR:BUSY");
        return;
    }

    if (strcmp(Command, "PING") == 0)
    {
        Serial_SendString("PONG");
    }
    else if (strcmp(Command, "DROP:1") == 0)
    {
        Servo_DoAction(SERVO_DROP1_ANGLE, "ACK:DROP:1");
    }
    else if (strcmp(Command, "DROP:2") == 0)
    {
        Servo_DoAction(SERVO_DROP2_ANGLE, "ACK:DROP:2");
    }
    else if (strcmp(Command, "DROP:3") == 0)
    {
        Servo_DoAction(SERVO_DROP3_ANGLE, "ACK:DROP:3");
    }
    else if (strcmp(Command, "LOCK") == 0)
    {
        Servo_DoAction(SERVO_LOCK_ANGLE, "ACK:LOCK");
    }
    else
    {
        Serial_SendString("ERR:INVALID");
    }
}

static void Servo_DoAction(float TargetAngle, const char *AckString)
{
    ServoBusy = 1;
    Angle = TargetAngle;
    Servo_SetAngle(TargetAngle);
    OLED_ShowNum(1, 7, (uint32_t)Angle, 3);
    Servo_ActionDelay(SERVO_ACTION_DELAY_MS);
    ServoBusy = 0;
    Serial_SendString(AckString);
}

static void Servo_ActionDelay(uint16_t DelayMs)
{
    while (DelayMs >= 10)
    {
        Serial_ProcessRx();
        Delay_ms(10);
        DelayMs -= 10;
    }

    while (DelayMs > 0)
    {
        Serial_ProcessRx();
        Delay_ms(1);
        DelayMs--;
    }
}

int main(void)
{
    OLED_Init();
    Servo_Init();
    Key_Init();
    Serial_Init();

    Angle = SERVO_LOCK_ANGLE;
    Servo_SetAngle(Angle);

    OLED_ShowString(1, 1, "Angle:");
    OLED_ShowString(2, 1, "UART 115200");
    OLED_ShowNum(1, 7, (uint32_t)Angle, 3);

    while (1)
    {
        Serial_ProcessRx();

        KeyNum = Key_GetNum();
        if (KeyNum == 1)
        {
            Angle += 30;
            if (Angle > 180)
            {
                Angle = 0;
            }

            Servo_SetAngle(Angle);
            OLED_ShowNum(1, 7, (uint32_t)Angle, 3);
        }
    }
}
