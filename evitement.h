#ifndef EVITEMENT_H
#define EVITEMENT_H

#include "tests.h"

#ifndef TEST
#include "position_manager.h"
#endif 

#include "obstacle.h"

typedef struct chemin chemin;
struct chemin {
  struct pos target;
  chemin* next;
};

#define SET_XY(pos, _x, _y) \
	pos.x = _x; pos.y = _y; pos.a = 0

chemin* evitement_get_chemin(struct obstacle obs[], uint8_t obs_num, struct pos* cur, struct pos* target); 
#endif//EVITEMENT_H
