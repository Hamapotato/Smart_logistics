#include "move.h"

Location_InitTypeDef Location;

u8 Step, Occupy_Flag;
u8 TIM_CNT = 0;
/* ms time base and the 30ms consumer flag, both driven by the TIM7 ISR.
   Needed from power-up (the throw arm homes before the run starts), which
   is why TIM7_Init() is called from main() and not from Task_Start(). */
volatile u32 TIM7_Tick_MS    = 0;
volatile u8  TIM7_TICK_FLAG  = 0;
u8           TASK_RUN_FLAG   = 0;

u8 ROUTE_PLAN_FLAG;
u8 k    = 0;
u8 FLAG = 0;
// float data[2];

void TIM7_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
    // 5ms计时
    TIM_TimeBaseStructure.TIM_Period        = 500 - 1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Prescaler     = 840 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);

    TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE); // 使能更新中断

    NVIC_InitStructure.NVIC_IRQChannel                   = TIM7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 2;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM7, ENABLE); // 使能TIM7
}

void Task_Start(void)
{
    TASK_RUN_FLAG = 1;
#if ARM_HOME_ON_START
    /* Re-reference the throw arm before the robot moves.  Steering_Home()
       raises STEERING_FLAG, and move.c's position-increment interlock keeps
       the robot standing until the arm reports done. */
    Steering_Home();
#endif
}

void Control_Data_Process(void)
{
    if (!BLUE_FLAG) {
        Location.X_Start = 0;
        Location.Y_Start = 0;
        Location.X_End   = 7;
        Location.Y_End   = 4;
        Location.X_Now   = Location.X_Start;
        Location.Y_Now   = Location.Y_Start;
        Location.X_Aim   = Location.X_Now;
        Location.Y_Aim   = Location.Y_Now;
        Step             = 0;
        Occupy_Flag      = 0;
        Direction        = STOP;
        BLUE_FLAG        = !BLUE_FLAG;
        Steering_RESET();
        Coordinate_Init();
        Coordinate_Refresh();
        ROUTE_PLAN_FLAG = Route_Plan();
    }
    if (!RED_FLAG) {
        Location.X_Start = 7;
        Location.Y_Start = 0;
        Location.X_End   = 0;
        Location.Y_End   = 4;
        Location.X_Now   = Location.X_Start;
        Location.Y_Now   = Location.Y_Start;
        Location.X_Aim   = Location.X_Now;
        Location.Y_Aim   = Location.Y_Now;
        Step             = 0;
        Occupy_Flag      = 0;
        Direction        = STOP;
        RED_FLAG         = !RED_FLAG;
        Steering_RESET();
        Coordinate_Init();
        Coordinate_Refresh();
        ROUTE_PLAN_FLAG = Route_Plan();
    }
    if (ROUTE_PLAN_FLAG) {
        if ((GREY.FRONT_SENSOR & GREY.LEFT_SENSOR) || (GREY.FRONT_SENSOR & GREY.RIGHT_SENSOR) || (GREY.BACK_SENSOR & GREY.LEFT_SENSOR) || (GREY.BACK_SENSOR & GREY.RIGHT_SENSOR)) {
            if (FRONT_L_OBSTACLE || FRONT_R_OBSTACLE) OBSTACLE.FRONT_OBSTACLE_COUNT++;
            if (BACK_L_OBSTACLE || BACK_R_OBSTACLE) OBSTACLE.BACK_OBSTACLE_COUNT++;
            if (LEFT_U_OBSTACLE || LEFT_D_OBSTACLE) OBSTACLE.LEFT_OBSTACLE_COUNT++;
            if (RIGHT_U_OBSTACLE || RIGHT_D_OBSTACLE) OBSTACLE.RIGHT_OBSTACLE_COUNT++;
        }
        if (((Direction == FRONT) && ((GREY.RIGHT_SENSOR | GREY.LEFT_SENSOR) & 0x18)) ||
            ((Direction == BACK) && ((GREY.RIGHT_SENSOR | GREY.LEFT_SENSOR) & 0x18)) ||
            ((Direction == LEFT) && ((GREY.FRONT_SENSOR | GREY.BACK_SENSOR) & 0x18)) ||
            ((Direction == RIGHT) && ((GREY.FRONT_SENSOR | GREY.BACK_SENSOR) & 0x18)) ||
            ((Direction == STOP) && ((GREY.FRONT_SENSOR | GREY.BACK_SENSOR | GREY.LEFT_SENSOR | GREY.RIGHT_SENSOR) & 0x18))) {
            if ((!Occupy_Flag) && (!STEERING_FLAG)) {
                if (Direction == RIGHT) Location.X_Now++;
                if (Direction == LEFT) Location.X_Now--;
                if (Direction == FRONT) Location.Y_Now++;
                if (Direction == BACK) Location.Y_Now--;
                Occupy_Flag = 1;
                if ((Location.X_Now == Location.X_Aim) && (Location.Y_Now == Location.Y_Aim)) {
                    if ((Location.X_Now == Location.X_End) && (Location.Y_Now == Location.Y_End)) {
                        Direction        = STOP;
                        Location.X_End   = Location.X_Start;
                        Location.Y_End   = Location.Y_Start;
                        Location.X_Start = Location.X_Now;
                        Location.Y_Start = Location.Y_Now;
                        Location.X_Aim   = Location.X_Now;
                        Location.Y_Aim   = Location.Y_Now;
                        Step             = 0;
                        Coordinate_Refresh();
                        ROUTE_PLAN_FLAG = Route_Plan();
                        Occupy_Flag     = 0;
                        Steering_SET(); /* 到达终点：发出抛投请求 */
                    } else {
                        Step++;

                        {
                            if (OBSTACLE.FRONT_OBSTACLE_COUNT > 0)
                                OBSTACLE.FRONT_OBSTACLE_COUNT = 1;
                            else
                                OBSTACLE.FRONT_OBSTACLE_COUNT = 0;
                            if (OBSTACLE.BACK_OBSTACLE_COUNT > 0)
                                OBSTACLE.BACK_OBSTACLE_COUNT = 1;
                            else
                                OBSTACLE.BACK_OBSTACLE_COUNT = 0;
                            if (OBSTACLE.LEFT_OBSTACLE_COUNT > 0)
                                OBSTACLE.LEFT_OBSTACLE_COUNT = 1;
                            else
                                OBSTACLE.LEFT_OBSTACLE_COUNT = 0;
                            if (OBSTACLE.RIGHT_OBSTACLE_COUNT > 0)
                                OBSTACLE.RIGHT_OBSTACLE_COUNT = 1;
                            else
                                OBSTACLE.RIGHT_OBSTACLE_COUNT = 0;
                            OBSTACLE.OBSTACLE_INFORMATION = (OBSTACLE.FRONT_OBSTACLE_COUNT << 3) | (OBSTACLE.BACK_OBSTACLE_COUNT << 2) | (OBSTACLE.LEFT_OBSTACLE_COUNT << 1) | (OBSTACLE.RIGHT_OBSTACLE_COUNT);
                        }

                        {
                            OBSTACLE.FRONT_OBSTACLE_COUNT = 0;
                            OBSTACLE.BACK_OBSTACLE_COUNT  = 0;
                            OBSTACLE.LEFT_OBSTACLE_COUNT  = 0;
                            OBSTACLE.RIGHT_OBSTACLE_COUNT = 0;
                        }

                        if (Location.X_Now == 0) OBSTACLE.OBSTACLE_INFORMATION = OBSTACLE.OBSTACLE_INFORMATION & 0x0D;
                        if (Location.X_Now == 7) OBSTACLE.OBSTACLE_INFORMATION = OBSTACLE.OBSTACLE_INFORMATION & 0x0E;
                        if (Location.Y_Now == 0) OBSTACLE.OBSTACLE_INFORMATION = OBSTACLE.OBSTACLE_INFORMATION & 0x0B;
                        if (Location.Y_Now == 4) OBSTACLE.OBSTACLE_INFORMATION = OBSTACLE.OBSTACLE_INFORMATION & 0x07;

                        {
                            Obstacle_Process();
                            if (Coordinate[Route[Step].X][Route[Step].Y].TRANSBILITY == UNTRANSIABILITY) {
                                Coordinate_Refresh();
                                ROUTE_PLAN_FLAG = Route_Plan();
                                Step            = 0;
                            }
                        }

                        Location.X_Aim = Route[Step].X;
                        Location.Y_Aim = Route[Step].Y;
                    }
                } else {
                    k++;
                    Direction       = STOP;
                    ROUTE_PLAN_FLAG = 0;
                    Location.X_Aim = Location.X_Now;
                    Location.Y_Aim = Location.Y_Now;
                    Step           = 0;
                    Occupy_Flag    = 0;
                    Coordinate_Init();
                    Coordinate_Refresh();
                    ROUTE_PLAN_FLAG = Route_Plan();
                }

                {
                    if ((Location.X_Aim - Location.X_Now) > 0)
                        Direction = RIGHT;
                    else if ((Location.X_Aim - Location.X_Now) < 0)
                        Direction = LEFT;
                    else if ((Location.Y_Aim - Location.Y_Now) > 0)
                        Direction = FRONT;
                    else if ((Location.Y_Aim - Location.Y_Now) < 0)
                        Direction = BACK;
                    else {
                        Direction = STOP;
                        k++;
                    }
                }
            }
        } else {
            if ((Direction == FRONT) && (FRONT_L_OBSTACLE || FRONT_R_OBSTACLE)) {
                Coordinate[Location.X_Now][Location.Y_Now + 1].TRANSBILITY = UNTRANSIABILITY;
                Coordinate_Refresh();
                ROUTE_PLAN_FLAG = Route_Plan();
                Location.X_Aim  = Location.X_Now;
                Location.Y_Aim  = Location.Y_Now;
                Location.X_Now  = Location.X_Now;
                Location.Y_Now  = Location.Y_Now + 1;
                Step            = 0;
                Direction       = BACK;
            } else if ((Direction == BACK) && (BACK_L_OBSTACLE || BACK_R_OBSTACLE)) {
                Coordinate[Location.X_Now][Location.Y_Now - 1].TRANSBILITY = UNTRANSIABILITY;
                Coordinate_Refresh();
                ROUTE_PLAN_FLAG = Route_Plan();
                Location.X_Aim  = Location.X_Now;
                Location.Y_Aim  = Location.Y_Now;
                Location.X_Now  = Location.X_Now;
                Location.Y_Now  = Location.Y_Now - 1;
                Step            = 0;
                Direction       = FRONT;
            } else if ((Direction == LEFT) && (LEFT_U_OBSTACLE || LEFT_D_OBSTACLE)) {
                Coordinate[Location.X_Now - 1][Location.Y_Now].TRANSBILITY = UNTRANSIABILITY;
                Coordinate_Refresh();
                ROUTE_PLAN_FLAG = Route_Plan();
                Location.X_Aim  = Location.X_Now;
                Location.Y_Aim  = Location.Y_Now;
                Location.X_Now  = Location.X_Now - 1;
                Location.Y_Now  = Location.Y_Now;
                Step            = 0;
                Direction       = RIGHT;
            } else if ((Direction == RIGHT) && (RIGHT_U_OBSTACLE || RIGHT_D_OBSTACLE)) {
                Coordinate[Location.X_Now + 1][Location.Y_Now].TRANSBILITY = UNTRANSIABILITY;
                Coordinate_Refresh();
                ROUTE_PLAN_FLAG = Route_Plan();
                Location.X_Aim  = Location.X_Now;
                Location.Y_Aim  = Location.Y_Now;
                Location.X_Now  = Location.X_Now + 1;
                Step            = 0;
                Direction       = LEFT;
            }
            if (((Direction == FRONT) && !((GREY.LEFT_SENSOR | GREY.RIGHT_SENSOR) & 0x18)) ||
                ((Direction == BACK) && !((GREY.LEFT_SENSOR | GREY.RIGHT_SENSOR) & 0x18)) ||
                ((Direction == LEFT) && !((GREY.FRONT_SENSOR | GREY.BACK_SENSOR) & 0x18)) ||
                ((Direction == RIGHT) && !((GREY.FRONT_SENSOR | GREY.BACK_SENSOR) & 0x18)) ||
                ((Direction == STOP) && !((GREY.FRONT_SENSOR | GREY.BACK_SENSOR | GREY.LEFT_SENSOR | GREY.RIGHT_SENSOR) & 0x18))) {
                Occupy_Flag                   = 0;
                OBSTACLE.FRONT_OBSTACLE_COUNT = 0;
                OBSTACLE.BACK_OBSTACLE_COUNT  = 0;
                OBSTACLE.LEFT_OBSTACLE_COUNT  = 0;
                OBSTACLE.RIGHT_OBSTACLE_COUNT = 0;
            }
        }
    } else {
        Direction      = STOP;
        Location.X_Aim = Location.X_Now;
        Location.Y_Aim = Location.Y_Now;
        Step           = 0;
        Occupy_Flag    = 0;
        Coordinate_Refresh();
        ROUTE_PLAN_FLAG = Route_Plan();
    }
}

void TIM7_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM7, TIM_IT_Update) == SET) {
        TIM7_Tick_MS += 5;
        if (TIM_CNT == 5) TIM7_TICK_FLAG = 1;
        /* Pre-start the ISR must stay off the grey buses: their drivers
           bit-bang with delay_us(), which reprograms SysTick and would
           corrupt any delay_ms() the main loop is inside of.  With the
           grey work gated out, the ISR only bumps the two counters. */
        if (TASK_RUN_FLAG) {
            if (TIM_CNT == 5) {

                // 灰度信息处理
                {
                    if (GREY.FRONT_SENSOR_COUNT > GREY_SENSOR_LIMITED)
                        GREY.FRONT_SENSOR = GREY.FRONT_SENSOR_INFORMATION;
                    else
                        GREY.FRONT_SENSOR = 0;
                    if (GREY.BACK_SENSOR_COUNT > GREY_SENSOR_LIMITED)
                        GREY.BACK_SENSOR = GREY.BACK_SENSOR_INFORMATION;
                    else
                        GREY.BACK_SENSOR = 0;
                    if (GREY.LEFT_SENSOR_COUNT > GREY_SENSOR_LIMITED)
                        GREY.LEFT_SENSOR = GREY.LEFT_SENSOR_INFORMATION;
                    else
                        GREY.LEFT_SENSOR = 0;
                    if (GREY.RIGHT_SENSOR_COUNT > GREY_SENSOR_LIMITED)
                        GREY.RIGHT_SENSOR = GREY.RIGHT_SENSOR_INFORMATION;
                    else
                        GREY.RIGHT_SENSOR = 0;
                }

                // 灰度清零
                {
                    GREY.FRONT_SENSOR_COUNT = 0;
                    GREY.BACK_SENSOR_COUNT  = 0;
                    GREY.LEFT_SENSOR_COUNT  = 0;
                    GREY.RIGHT_SENSOR_COUNT = 0;
                }

                Control_Data_Process();

                TIM_CNT = 0;
            } else {
                { // 获取灰度信息
                    GREY.FRONT_SENSOR_INFORMATION = ~F_GW_digital_Read();
                    GREY.BACK_SENSOR_INFORMATION  = ~B_GW_digital_Read();
                    GREY.LEFT_SENSOR_INFORMATION  = ~L_GW_digital_Read();
                    GREY.RIGHT_SENSOR_INFORMATION = ~R_GW_digital_Read();
                }

                { // 统计灰度信息
                    if (GREY.FRONT_SENSOR_INFORMATION & 0x18) GREY.FRONT_SENSOR_COUNT++;
                    if (GREY.BACK_SENSOR_INFORMATION & 0x18) GREY.BACK_SENSOR_COUNT++;
                    if (GREY.LEFT_SENSOR_INFORMATION & 0x18) GREY.LEFT_SENSOR_COUNT++;
                    if (GREY.RIGHT_SENSOR_INFORMATION & 0x18) GREY.RIGHT_SENSOR_COUNT++;
                }
                TIM_CNT++;
            }
        } else {
            TIM_CNT++; /* keep the divider alive so TIM7_TICK_FLAG keeps firing */
        }
        TIM_ClearITPendingBit(TIM7, TIM_IT_Update);

    }
}

void go(void)
{
    while(((GREY.BACK_SENSOR)&0xfc)||((GREY.BACK_SENSOR)&0xf8)||((GREY.BACK_SENSOR)&0x3F)||((GREY.BACK_SENSOR)&0x1f))
    {
        Motor_201.PID_Output = -5;
        Motor_202.PID_Output = -5;
        Motor_203.PID_Output = -5;
        Motor_204.PID_Output = -5;
        MOTOR_CONTROL_DATA_Predit(&Motor_201, &Motor_202, &Motor_203, &Motor_204);
    }
}

void ba(void)
{
    while(!((GREY.RIGHT_SENSOR|GREY.LEFT_SENSOR)&0x18))
    {
        Motor_201.PID_Output = 5;
        Motor_202.PID_Output = 5;
        Motor_203.PID_Output = 5;
        Motor_204.PID_Output = 5;
        MOTOR_CONTROL_DATA_Predit(&Motor_201, &Motor_202, &Motor_203, &Motor_204);
    }
}