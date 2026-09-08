/*
 * path_plan.h
 *
 *  Created on: 2026年7月20日
 *      Author: twyyd
 */

#ifndef INC_PATH_PLAN_H_
#define INC_PATH_PLAN_H_

#include "stdio.h"
#include "stdlib.h"
#include "stdint.h"
#include "usart.h"
#include "stdbool.h"

#define MAP_SIZE 5
#define MAP_END (MAP_SIZE - 1)
#define MAP_MID (MAP_END / 2)
#define MAP_OBS (MAP_MID / 2)
#define INF 9999

typedef struct{
	int x;
	int y;

	float real_x;
	float real_y;
	float real_r;

	int g;
	int h;
	int f;

	int parent_x;
	int parent_y;


	bool open;
	bool close;
}Node;

typedef struct
{
    uint8_t x;
    uint8_t y;

}PathPoint;


int Manhattan(int x1,int y1,int x2,int y2);
void InitNodes();
int IsValid(int x,int y);
void Initreal();
Node* GetMinNode();
int AStar(int start_x,int start_y,int goal_x,int goal_y);
void GetPath(int x,int y);
void ReversePath();
bool Find_path(int start_x,int start_y,int goal_x,int goal_y);

#endif /* INC_PATH_PLAN_H_ */
