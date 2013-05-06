#include "obstacle.h"
#include <math.h>

#define MAX_CHEMIN 20
chemin buffer[MAX_CHEMIN] = {0};
uint8_t buff_size;

uint16_t dist(struct pos* p1, struct pos* p2) {
	uint16_t x, y;
	x = p1->x - p2->x;
	y = p1->y - p2->y;

	return sqrt(x*x+y*y);
}

#define SET_XYA(pos, _x, _y, _a) \
	pos.x = _x; pos.y = _y; pos.a = _a

#define DIST_PTS 30

chemin* evitement_get_chemin(struct obstacle obs[], uint8_t obs_num, struct pos* cur, struct pos* target) {
	buff_size = 0;
	SET_XYA(buffer[0].target, cur->x, cur->y,0);

	while(buff_size < MAX_CHEMIN && dist(&buffer[buff_size].target, target) > 30) {
		// Calcul du point le plus proche
		struct pos min;
		for(int i = -1 ; i < 2 ; i++) {
			for(int j = -1 ; j < 2 ; j++) {
				struct pos pts = {buffer[buff_size].target.x + i * DIST_PTS, buffer[buff_size].target.y + j * DIST_PTS};
				
			}
		}
	}
	
}
