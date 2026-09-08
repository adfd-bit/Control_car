/*
 * path_plan.c
 *
 *  Created on: 2026年7月20日
 *      Author: twyyd
 */
#include "path_plan.h"

// 0:可走
// 1:障碍

uint8_t map[MAP_SIZE][MAP_SIZE]={
		{0,0,0,0,0},
		{0,1,0,1,0},
		{0,0,0,0,0},
		{0,1,0,1,0},
		{0,0,0,0,0}
};
Node nodes[MAP_SIZE][MAP_SIZE];
int path_length;
PathPoint path[MAP_SIZE * MAP_SIZE];
int dir[4][2]={
		{1,0},
		{-1,0},
		{0,1},
		{0,-1}
};
int Manhattan(int x1,int y1,int x2,int y2){
	return abs(x1 - x2)+abs(y1 - y2);
}
void InitNodes(){
	for(int i=0;i<MAP_SIZE;i++){
		for(int j=0;j<MAP_SIZE;j++){

			nodes[i][j].x = i;
			nodes[i][j].y = j;

			nodes[i][j].real_x = 0.0;
			nodes[i][j].real_y = 0.0;
			nodes[i][j].real_r = 0.0;

			nodes[i][j].g = INF;
			nodes[i][j].h = 0;
			nodes[i][j].f = INF;

			nodes[i][j].parent_x = -1;
			nodes[i][j].parent_y = -1;

			nodes[i][j].open = false;
			nodes[i][j].close = false;
		}
	}
}
void Initreal(){
	nodes[0][0].real_x = 200;
	nodes[0][0].real_y = 200;

	nodes[MAP_MID][0].real_x = 1000;
	nodes[MAP_MID][0].real_y = 200;

	nodes[MAP_END][0].real_x = 1900;
	nodes[MAP_END][0].real_y = 200;

	nodes[0][MAP_MID].real_x = 200;
	nodes[0][MAP_MID].real_y = 1050;

	nodes[0][MAP_END].real_x = 200;
	nodes[0][MAP_END].real_y = 1900;

	nodes[MAP_MID][MAP_MID].real_x = 1000;
	nodes[MAP_MID][MAP_MID].real_y = 1050;

	nodes[MAP_MID][MAP_END].real_x = 1000;
	nodes[MAP_MID][MAP_END].real_y = 1900;

	nodes[MAP_END][MAP_MID].real_x = 1900;
	nodes[MAP_END][MAP_MID].real_y = 1050;

	nodes[MAP_END][MAP_END].real_x = 1900;
	nodes[MAP_END][MAP_END].real_y = 1900;
}
int IsValid(int x,int y){
	if(x < 0 || x >= MAP_SIZE)
		return 0;
	if(y < 0 || y >= MAP_SIZE)
		return 0;
	if(map[x][y] == 1)
		return 0;

	return 1;
}

Node* GetMinNode(){
	Node* min =NULL;
	Node* n;
	for(int i=0;i<MAP_SIZE;i++){
		for(int j=0;j<MAP_SIZE;j++){
			n = &nodes[i][j];
			if(n->open && !n->close){
				if(min == NULL || n->f < min->f){
					min = n;
				}
			}
		}
	}
	return min;
}
int AStar(int start_x,int start_y,int goal_x,int goal_y){
	InitNodes();
	Initreal();
	Node* current;
	Node* next ;
	Node* start = &nodes[start_x][start_y];
	start->g = 0;
	start->h = Manhattan(start_x, start_y, goal_x, goal_y);
	start->f = start->g + start->h;
	start->open = true;
	while(1){
		current = GetMinNode();
		if(current == NULL){//无有效路径
			return 0;
		}
		if(current->x == goal_x && current->y == goal_y){//到达目标
			return 1;
		}
		current->close = true;
		for(int i = 0;i<4;i++){
			int nx = current->x + dir[i][0];
			int ny = current->y + dir[i][1];
			if(!IsValid(nx, ny))
				continue;
			next = &nodes[nx][ny];
			if(next->close)
				continue;
			int new_g = current->g + 1;
			if(!next->open || new_g < next->g){
				next->g = new_g;
				next->h = Manhattan(nx, ny, goal_x, goal_y);
				next->f = next->g + next->h;
				next->parent_x = current->x;
				next->parent_y = current->y;
				next->open = true;
			}
		}
	}
}
void GetPath(int x,int y){
	int py;
	int px;
	path_length = 0;
	while(1){
		path[path_length].x = x;
		path[path_length].y = y;
		path_length++;
		if(nodes[x][y].parent_x == -1)
			break;
		px = nodes[x][y].parent_x;
		py = nodes[x][y].parent_y;
		x = px;
		y = py;
	}
}
void ReversePath(){
	for(int i= 0;i<path_length/2;i++){
		PathPoint temp = path[i];
		path[i] = path[path_length-1-i];
		path[path_length-1-i] = temp;
	}
}
bool Find_path(int start_x,int start_y,int goal_x,int goal_y){
	if(AStar(start_x, start_y, goal_x, goal_y)){
		GetPath(goal_x, goal_y);
		ReversePath();
		return true;
	}
	else{
//		//注意删除
//		HAL_UART_Transmit(&huart6, (uint8_t*)("NULL"), sizeof("NULL"), 10000);
		return false;
	}
}
