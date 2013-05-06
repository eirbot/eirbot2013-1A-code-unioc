#include "strategie.h"

#include <stdio.h>


#define SET_POS(pos, _x, _y, _a)			\
  pos.x = _x; pos.a = _y; pos.a = _a

void strategie_init(struct strategie* strat, enum cote beg) {
  position_init(&strat->pos);
  
  // Initialisation de la position de départ
  strat->side = beg;
  if(beg == COTE_ROUGE) {
    position_begin_set(&strat->pos, 140, 100, 180);
  }
  else if(beg == COTE_BLEU) {
    position_begin_set(&strat->pos, -140, 100, 0);
  }
  else {
    printf("ERREUR : dans l'initialisation du depart");
  }

  // Date
  strat->cur_date = 0;

  // Initialisation de la position des autres robots
  obstacle empty_obs;
  MK_CIRCLE(empty_obs, 0,0,0);
  for(int i = 0 ; i < OTHER_ROBOTS ; i++) {
    strat->other[i] = empty_obs;
    strat->last_seen[i] = 0;
  }

  // Position des verres
  glassManagerInit(&strat->gm);

  // Objectif
  strat->cur_objectif = NULL;
}

// Donne le prochain endroit où aller
void strategie_where(struct strategie* strat, uint16_t* x, uint16_t* y, uint16_t* a) {
  if(strategie_arrived(strat))
    return;
  *x = strat->cur_objectif->target.x;
  *y = strat->cur_objectif->target.y;
  *a = strat->cur_objectif->target.a;
}

// Dit si le prochain objectif a été atteint
bool strategie_arrived(struct strategie* strat)
{
  return (NULL == strat->cur_objectif);
}

// Retourne le mode de strategie
enum strategie_mode strategie_mode(struct strategie* strat) {
  return strat->mode;
}
// Pour prendre en compte le retour des évènements
void strategie_update_gp2(struct strategie* strat, uint16_t adc_ga, uint16_t adc_dr, uint16_t adc_av, uint16_t adc_ar) {
  int16_t x,y,a;
  if(gp2_get_dist(strat->gp2[0], adc_ga) < 0) //gp2_too_far(strat->gp2)) {
  {
	  gp2_get_obstacle(strat->gp2[0], adc_ga, &x, &y, &a);
    //known_obstacle(x, y, a);
  }

}
void strategie_update_meca(struct strategie* strat);

// Cherche le prochain objectif
void strategie_fetch(struct strategie* strat);
