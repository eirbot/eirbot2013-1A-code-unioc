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
  for(int i = 0 ; i < OTHER_ROBOTS ; i++) {
    SET_POS(strat->other[i], 0,0,0);
    strat->last_seen[i] = 0;
  }

  // Position des verres
  glassManagerInit(&strat->gm);

  // Objectif
  strat->cur_objectif = NULL;
}

// Donne le prochain endroit où aller
void strategie_where(struct strategie* strat, uint16_t* x, uint16_t* y);

// Dit si le prochain objectif a été atteint
bool strategie_arrived(struct strategie* strat);

// Retourne le mode de strategie
enum strategie_mode strategie_mode(struct strategie* strat);

// Pour prendre en compte le retour des évènements
void strategie_update_gp2(struct strategie* strat, uint16_t adc_ga, uint16_t adc_dr, uint16_t adc_av, uint16_t adc_ar);
void strategie_update_meca(struct strategie* strat);

// Cherche le prochain objectif
void strategie_fetch(struct strategie* strat);
