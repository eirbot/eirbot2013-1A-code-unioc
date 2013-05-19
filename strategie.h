#ifndef STRATEGIE_H
#define STRATEGIE_H

#include <aversive.h>

#include "verre.h"
#include "position_manager.h"
#include "obstacle.h"
#include "evitement.h"
#include "gp2.h"

#define OTHER_ROBOTS 3
#define TIMER_FORGET 4000
// 
enum strategie_mode {
  SEARCH_GLASS_MODE,
  SEARCH_RANDOM_MODE,
  GO_BACK_MODE,
  PUT_STACK_MODE,

  MAX_MODE
};

enum cote {
  COTE_BLEU,
  COTE_ROUGE,
  COTE_NONE
};

enum gp2 {
  GP2_AVANT,
  GP2_GAUCHE,
  GP2_DROITE,
  GP2_ARRIERE,
  GP2_MAX
};

struct strategie {
  struct gp2_cfg_t* gp2[GP2_MAX]; 
  // Cote de depart
  enum cote side;
 // Dernière position des autres robots
  struct obstacle other[OTHER_ROBOTS];
  // Dernière date de détection
  uint32_t last_seen[OTHER_ROBOTS];
  uint32_t cur_date;
  // Position de départ des verres
  glass_manager gm;
  // Objectif
  chemin* cur_objectif;
  // Mode de strategie
  enum strategie_mode mode;
  glass_t* verre;
  // Position actuelle
  position_manager_t pos;
};

#define bool uint8_t
#define TRUE 1
#define FALSE 0
#define ROBOT_RAYON 25

// Initialisation
void strategie_init(struct strategie* strat, enum cote beg);
void robot_evitement(struct strategie* strat, int16_t x, int16_t y, int16_t a);
uint8_t robot_obstacle(struct strategie* strat, int16_t x, int16_t y);
// Donne le prochain endroit où aller
void strategie_where(struct strategie* strat, int16_t* x, int16_t* y, int16_t* a);

// Dit si le prochain objectif a été atteint
bool strategie_arrived(struct strategie* strat);

// Retourne le mode de strategie
enum strategie_mode strategie_mode(struct strategie* strat);

// Pour prendre en compte le retour des évènements
void strategie_update_gp2(struct strategie* strat, uint16_t adc_ga, uint16_t adc_dr, uint16_t adc_av, uint16_t adc_ar);
void strategie_update_meca(struct strategie* strat);

// Cherche le prochain objectif
void strategie_fetch(struct strategie* strat);

// Edit the color
void strategie_set_color(struct strategie* strat, enum cote side);

#endif//STRATEGIE_H
