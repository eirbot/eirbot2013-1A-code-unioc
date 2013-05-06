#ifndef EVITEMENT_H
#define EVITEMENT_H

typedef struct chemin chemin;
struct chemin {
  struct pos target;
  chemin* next;
};

chemin* evitement_get_chemin(struct obstacle obs[], uint8_t obs_num, int16_t x, int16_t y); 

#endif//EVITEMENT_H
