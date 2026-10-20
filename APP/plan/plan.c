#include "plan.h"

Coordinate_InitTypeDef Coordinate[8][5];
Route_InitTypeDef Route[40];

void Coordinate_Init(void)
{
    for (int8_t i = 0; i < 8; i++) {
        for (int8_t j = 0; j < 5; j++) {
            Coordinate[i][j].TRANSBILITY = TRANSIABILITY;
            Coordinate[i][j].Cost = 0;
            Coordinate[i][j].End_To_Start = 0;
            Coordinate[i][j].Start_To_End = 0;
        }
    }
}

void Coordinate_Refresh(void)
{
    for (int8_t i = 0; i < 8; i++) {
        for (int8_t j = 0; j < 5; j++) {
            Coordinate[i][j].Start_To_End = INFINISS;
            Coordinate[i][j].End_To_Start = INFINISS;
            Coordinate[i][j].Cost = 0;
        }
    }

    Coordinate[Location.X_Now][Location.Y_Now].Start_To_End = 0;
    Coordinate[Location.X_End][Location.Y_End].End_To_Start = 0;

    for (int8_t f = 0; f < 60; f++) {
        for (int8_t i = 0; i < 8; i++) {
            for (int8_t j = 0; j < 5; j++) {
                if (Coordinate[i][j].Start_To_End == f) {
                    if ((i>0) && (Coordinate[i-1][j].TRANSBILITY != UNTRANSIABILITY) && (Coordinate[i-1][j].Start_To_End > f)) Coordinate[i-1][j].Start_To_End=f+1;
                    if ((i<7) && (Coordinate[i+1][j].TRANSBILITY != UNTRANSIABILITY) && (Coordinate[i+1][j].Start_To_End > f)) Coordinate[i+1][j].Start_To_End=f+1;
                    if ((j>0) && (Coordinate[i][j-1].TRANSBILITY != UNTRANSIABILITY) && (Coordinate[i][j-1].Start_To_End > f)) Coordinate[i][j-1].Start_To_End=f+1;
                    if ((j<4) && (Coordinate[i][j+1].TRANSBILITY != UNTRANSIABILITY) && (Coordinate[i][j+1].Start_To_End > f)) Coordinate[i][j+1].Start_To_End=f+1;
                }
                if (Coordinate[i][j].End_To_Start == f) {
                    if ((i>0) && (Coordinate[i-1][j].TRANSBILITY != UNTRANSIABILITY) && (Coordinate[i-1][j].End_To_Start > f)) Coordinate[i-1][j].End_To_Start=f+1;
                    if ((i<7) && (Coordinate[i+1][j].TRANSBILITY != UNTRANSIABILITY) && (Coordinate[i+1][j].End_To_Start > f)) Coordinate[i+1][j].End_To_Start=f+1;
                    if ((j>0) && (Coordinate[i][j-1].TRANSBILITY != UNTRANSIABILITY) && (Coordinate[i][j-1].End_To_Start > f)) Coordinate[i][j-1].End_To_Start=f+1;
                    if ((j<4) && (Coordinate[i][j+1].TRANSBILITY != UNTRANSIABILITY) && (Coordinate[i][j+1].End_To_Start > f)) Coordinate[i][j+1].End_To_Start=f+1;
                }
            }
        }
        if ((Coordinate[Location.X_End][Location.Y_End].Start_To_End < INFINISS) && (Coordinate[Location.X_Now][Location.Y_Now].End_To_Start < INFINISS)) break;
    }

    for (int8_t i = 0; i < 8; i++) {
        for (int8_t j = 0; j < 5; j++) {
            Coordinate[i][j].Cost = Coordinate[i][j].Start_To_End + Coordinate[i][j].End_To_Start;
        }
    }
}

u8 Route_Plan(void)
{
    short step=0;
    Route[0].X = Location.X_Now;
    Route[0].Y = Location.Y_Now;

    if (Coordinate[Location.X_End][Location.Y_End].Start_To_End > 50) {
        printf("Coordinate_Refresh_failed:%d\r\n",step);
        return 0;
    }

    for (step = 1; step < Coordinate[Location.X_End][Location.Y_End].Start_To_End + 1; step++) {
        if (ROUTE_CHOSE && (Route[step - 1].Y < 4) && (Coordinate[Route[step - 1].X][Route[step - 1].Y + 1].Start_To_End == step) && (Coordinate[Route[step - 1].X][Route[step - 1].Y + 1].Cost == Coordinate[Route[step - 1].X][Route[step - 1].Y].Cost)) {
            Route[step].X = Route[step - 1].X;
            Route[step].Y = Route[step - 1].Y + 1;
        } else if ((Route[step - 1].X < 7) && (Coordinate[Route[step - 1].X + 1][Route[step - 1].Y].Start_To_End == step) && (Coordinate[Route[step - 1].X + 1][Route[step - 1].Y].Cost == Coordinate[Route[step - 1].X][Route[step - 1].Y].Cost)) {
            Route[step].X = Route[step - 1].X + 1;
            Route[step].Y = Route[step - 1].Y;
        } else if ((Route[step - 1].Y > 0) && (Coordinate[Route[step - 1].X][Route[step - 1].Y - 1].Start_To_End == step) && (Coordinate[Route[step - 1].X][Route[step - 1].Y - 1].Cost == Coordinate[Route[step - 1].X][Route[step - 1].Y].Cost)) {
            Route[step].X = Route[step - 1].X;
            Route[step].Y = Route[step - 1].Y - 1;
        } else if ((Route[step - 1].X > 0) && (Coordinate[Route[step - 1].X - 1][Route[step - 1].Y].Start_To_End == step) && (Coordinate[Route[step - 1].X - 1][Route[step - 1].Y].Cost == Coordinate[Route[step - 1].X][Route[step - 1].Y].Cost)) {
            Route[step].X = Route[step - 1].X - 1;
            Route[step].Y = Route[step - 1].Y;
        } else if ((Route[step - 1].Y < 4) && (Coordinate[Route[step - 1].X][Route[step - 1].Y + 1].Start_To_End == step) && (Coordinate[Route[step - 1].X][Route[step - 1].Y + 1].Cost == Coordinate[Route[step - 1].X][Route[step - 1].Y].Cost)) {
            Route[step].X = Route[step - 1].X;
            Route[step].Y = Route[step - 1].Y + 1;
        }
    }

    if (Route[Coordinate[Location.X_End][Location.Y_End].Start_To_End].X == Location.X_End && Route[Coordinate[Location.X_End][Location.Y_End].Start_To_End].Y == Location.Y_End) {
        return 1;
    } else {
        printf("Route_Plan_failed:%d\r\n",step);
        return 0;
    }
}

void Obstacle_Process(void)
{
    if(Location.X_Now < 7) Coordinate[Location.X_Now + 1][Location.Y_Now].TRANSBILITY = OBSTACLE.OBSTACLE_INFORMATION&0x01;
    if(Location.X_Now > 0) Coordinate[Location.X_Now - 1][Location.Y_Now].TRANSBILITY = (OBSTACLE.OBSTACLE_INFORMATION>>1)&0x01;
    if(Location.Y_Now > 0) Coordinate[Location.X_Now][Location.Y_Now - 1].TRANSBILITY = (OBSTACLE.OBSTACLE_INFORMATION>>2)&0x01;
    if(Location.Y_Now < 3) Coordinate[Location.X_Now][Location.Y_Now + 1].TRANSBILITY = (OBSTACLE.OBSTACLE_INFORMATION>>3)&0x01;

    if((Location.X_End == 7) && (Location.Y_End == 0)) Coordinate[7][0].TRANSBILITY = UNTRANSIABILITY;
    if((Location.X_End == 0) && (Location.Y_End == 3)) Coordinate[0][3].TRANSBILITY = UNTRANSIABILITY;
}