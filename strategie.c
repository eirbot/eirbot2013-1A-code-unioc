#include "strategie.h"
//#include "i2cm.h"
//#include "com.h"
#include "evitement.h"

#include <stdio.h>

#define DISTANCE_MIN_EXT 50
#define DISTANCE_MIN_INT 20
#define DISTANCE_MIN_ARR 10
#define SET_POS(pos, _x, _y, _a)			\
  pos.x = _x; pos.a = _y; pos.a = _a

static struct chemin global_chemin;

void strategie_init(struct strategie* strat, enum cote beg) {
  position_init(&strat->pos);

  	//// GP2 Avant Gauche
	gp2_init_xya(&strat->gp2[GP2_GAUCHE], 10, 12, 0);

	gp2_add_point(&strat->gp2[GP2_GAUCHE], 150,  500);
	gp2_add_point(&strat->gp2[GP2_GAUCHE], 120,  600);
	gp2_add_point(&strat->gp2[GP2_GAUCHE], 100,  700);
	gp2_add_point(&strat->gp2[GP2_GAUCHE],  60, 1050);
	gp2_add_point(&strat->gp2[GP2_GAUCHE],  50, 1250);
	gp2_add_point(&strat->gp2[GP2_GAUCHE],  45, 1375);
	gp2_add_point(&strat->gp2[GP2_GAUCHE],  40, 1500);
	gp2_add_point(&strat->gp2[GP2_GAUCHE],  30, 2000);
	gp2_add_point(&strat->gp2[GP2_GAUCHE],  20, 2500);

	// GP2 Avant Droite
	gp2_init_xya(&strat->gp2[GP2_DROITE], 10, -12, 0);

	gp2_add_point(&strat->gp2[GP2_DROITE], 150,  500);
	gp2_add_point(&strat->gp2[GP2_DROITE], 120,  600);
	gp2_add_point(&strat->gp2[GP2_DROITE], 100,  700);
	gp2_add_point(&strat->gp2[GP2_DROITE],  60, 1050);
	gp2_add_point(&strat->gp2[GP2_DROITE],  50, 1250);
	gp2_add_point(&strat->gp2[GP2_GAUCHE],  45, 1375);
	gp2_add_point(&strat->gp2[GP2_DROITE],  40, 1500);
	gp2_add_point(&strat->gp2[GP2_DROITE],  30, 2000);
	gp2_add_point(&strat->gp2[GP2_DROITE],  20, 2500);

	// GP2 Avant Millieu
	gp2_init_xya(&strat->gp2[GP2_AVANT], 18, 0 ,0);

	gp2_add_point(&strat->gp2[GP2_AVANT],  80, 400);
	gp2_add_point(&strat->gp2[GP2_AVANT],  70, 450);
	gp2_add_point(&strat->gp2[GP2_AVANT],  60, 550);
	gp2_add_point(&strat->gp2[GP2_AVANT],  50, 625);
	gp2_add_point(&strat->gp2[GP2_AVANT],  40, 760);
	gp2_add_point(&strat->gp2[GP2_AVANT],  30, 975);
	gp2_add_point(&strat->gp2[GP2_AVANT],  20, 1400);
	gp2_add_point(&strat->gp2[GP2_AVANT],  10, 2500);
	// GP2 Arriere
	gp2_init_xya(&strat->gp2[GP2_ARRIERE], 18, 0 ,0);

	gp2_add_point(&strat->gp2[GP2_ARRIERE],  80, 400);
	gp2_add_point(&strat->gp2[GP2_ARRIERE],  70, 450);
	gp2_add_point(&strat->gp2[GP2_ARRIERE],  60, 550);
	gp2_add_point(&strat->gp2[GP2_ARRIERE],  50, 625);
	gp2_add_point(&strat->gp2[GP2_ARRIERE],  40, 760);
	gp2_add_point(&strat->gp2[GP2_ARRIERE],  30, 975);
	gp2_add_point(&strat->gp2[GP2_ARRIERE],  20, 1400);
	gp2_add_point(&strat->gp2[GP2_ARRIERE],  10, 2500);
		
  // Initialisation de la position de départ
  strat->side = beg;
  if(beg == COTE_ROUGE) {
    position_begin_set(&strat->pos, -140, 100, 180);
  }
  else if(beg == COTE_BLEU) {
    position_begin_set(&strat->pos, 140, 100, 0);
  }
  else {
    printf("ERREUR : dans l'initialisation du depart");
  }
//verre 
	strat->verre = NULL;
  // Date
  strat->cur_date = 0;

  // Initialisation de la position des autres robots
  for(int i = 0 ; i < OTHER_ROBOTS ; i++) {
  	MK_CIRCLE(strat->other[i], 0,0,0);
    strat->last_seen[i] = 0;
  }

  // Position des verres
  glassManagerInit(&strat->gm);

  // Objectif
  strat->cur_objectif = NULL;
  //strategie_fetch(strat);
}

// Donne le prochain endroit où aller
void strategie_where(struct strategie* strat, int16_t* x, int16_t* y, int16_t* a) {
  if(strategie_arrived(strat))
    return;
  *x = strat->cur_objectif->target.x;
  *y = strat->cur_objectif->target.y;
  *a = 0;
}

// Dit si le prochain objectif a été atteint
bool strategie_arrived(struct strategie* strat)
{
  if(NULL == strat->cur_objectif)
	  return TRUE;
  int16_t robot_x,robot_y,robot_a;
  position_abs(&strat->pos, &robot_x, &robot_y, &robot_a);
  printf("(%i, %i) (%i, %i)\n", robot_x, robot_y, strat->cur_objectif->target.x, strat->cur_objectif->target.y);
  if(robot_x - strat->cur_objectif->target.x < 2 && robot_y - strat->cur_objectif->target.y < 2 && robot_x - strat->cur_objectif->target.x > -2 && robot_y - strat->cur_objectif->target.y > -2)
	  strat->cur_objectif = strat->cur_objectif->next;
  return (NULL == strat->cur_objectif);
}

// Retourne le mode de strategie
enum strategie_mode strategie_mode(struct strategie* strat) {
  return strat->mode;
}
// Pour prendre en compte le retour des évènements
void strategie_update_gp2(struct strategie* strat, uint16_t adc_ga, uint16_t adc_dr, uint16_t adc_av, uint16_t adc_ar) {
  int16_t x,y,a,robot;
  //if(gp2_get_dist(strat->gp2[GP2_AVANT], adc_av) < DISTANCE_MIN_INT) //gp2_too_far(strat->gp2)) {
  //{
  //	  gp2_get_obstacle(strat->gp2[GP2_AVANT], adc_av, &x, &y, &a);
  //	  robot_evitement(strat, x, y, a);
  //}
  if(adc_ga < DISTANCE_MIN_EXT) //gp2_too_far(strat->gp2)) {
	{
	  gp2_get_obstacle(strat->gp2[GP2_GAUCHE], adc_ga, &x, &y, &a);
	  robot_evitement(strat, x, y, a);
	}
  if(adc_dr < DISTANCE_MIN_EXT) //gp2_too_far(strat->gp2)) {
	{
	  gp2_get_obstacle(strat->gp2[GP2_DROITE], adc_dr, &x, &y, &a);
	  robot_evitement(strat, x, y, a);
	}
  /*
  if(gp2_get_dist(strat->gp2[GP2_ARRIERE], adc_ar) < DISTANCE_MIN_ARR) //gp2_too_far(strat->gp2)) {
	{
	  gp2_get_obstacle(strat->gp2[GP2_ARRIERE], adc_ar, &x, &y, &a);
	  //robot_evitement(strat, x, y, a);
	}
/**/
  strat->cur_date++;
  //clean robot
  for(robot = 1; robot <= OTHER_ROBOTS ; robot++)
  {
	if(strat->cur_date < strat->last_seen[robot-1])
	{
		if(0xffffffff - strat->last_seen[robot-1] + strat->cur_date > TIMER_FORGET)
	    {
		  MK_CIRCLE(strat->other[robot-1], 0, 0,0);
		  strat->last_seen[robot-1] = strat->cur_date;
	    }

	}
	else if(strat->cur_date - strat->last_seen[robot-1] > TIMER_FORGET)
	{
		MK_CIRCLE(strat->other[robot-1], 0, 0,0);
		strat->last_seen[robot-1] = strat->cur_date;
	}
 }

}

uint8_t robot_obstacle(struct strategie* strat, int16_t x, int16_t y)
 {
	for(int i=0;i<OTHER_ROBOTS;i++)
	{
		if(is_obstacle(&(strat->other[i]), x, y))
			return i+1;
	}
	return 0;
 }

void rel_to_abs(int16_t rel_x, int16_t rel_y, int16_t abs_x, int16_t abs_y, int16_t abs_a, int16_t *ret_x, int16_t *ret_y) {
  float c = cos(((float)abs_a * 3.14 / 180.0));
  float s = sin(((float)abs_a * 3.14 / 180.0));
  PRINT_DBG("angle %d ;; cos = %f , sin = %f", abs_a, c, s);
  *ret_x = c*(float)rel_x + s*(float)rel_y;
  *ret_y = - s*(float)rel_x + c*(float)rel_y;
  *ret_x = abs_x - (*ret_x);
  *ret_y = abs_y - (*ret_y);
}

void clean_traj(void);

void new_way(struct strategie* strat, struct pos* cur, struct pos* target) {
  strat->cur_objectif = evitement_get_chemin(strat->other, OTHER_ROBOTS, cur, target);
  if(strat->cur_objectif == NULL) {
    PRINT_DBG("Ne peut atteindre le point (%d, %d)", target->x, target->y);
    removeGlass(&(strat->gm), strat->verre);
    strat->verre = NULL;
    strategie_fetch(strat);
  }
  clean_traj();
}

void robot_evitement(struct strategie* strat, int16_t x, int16_t y, int16_t a) {
  int8_t robot;
  int16_t robot_x,robot_y,robot_a;
  int16_t ax, ay;

  struct pos cur, target;

  position_abs(&strat->pos, &robot_x, &robot_y, &robot_a);
  /*x += robot_x;
    y += robot_y;*/
  rel_to_abs(x, y, robot_x, robot_y, robot_a, &ax, &ay);

  SET_XY(cur, robot_x, robot_y);
  SET_XY(target, strat->cur_objectif->target.x, strat->cur_objectif->target.y);

  a = (a+robot_a+360)%360;
  PRINT_DBG("ROBOT DETECTE EN (%d, %d)\n", ax, ay);
  int evitement = 0;
  if((evitement = known_obstacle(ax, ay)))
    {
      //wait_ms(1000);
		  
      if(evitement != OBS_IGNORE){
	PRINT_DBG("Obstacle non ignoré %d\n", evitement);
	new_way(strat,&cur, &target);
      }
    }
  else if((robot = robot_obstacle(strat,ax,ay)))
    {
      //wait_ms(1000);
      PRINT_DBG("ROBOT !! \n\n");
      if(robot!= OTHER_ROBOTS) // pas deja le plus recent
	{
	  MK_COPY(strat->other[robot-1], strat->other[OTHER_ROBOTS-1]);
	  strat->last_seen[robot-1] = strat->last_seen[OTHER_ROBOTS-1];
	}	
      MK_CIRCLE(strat->other[robot-1], ax,ay, ROBOT_RAYON);
      strat->last_seen[robot-1] = strat->cur_date;
      new_way(strat, &cur, &target);
    }
  else
    {
      PRINT_DBG("AUTRE \n");
      //wait_ms(1000);
      for(robot = OTHER_ROBOTS; robot > 0 && strat->other[robot-1].dr != 0; robot--) PRINT_DBG("robot = %d", robot);	 
      MK_CIRCLE(strat->other[robot-1], ax,ay, ROBOT_RAYON);
      new_way(strat, &cur, &target);
    }
}

void strategie_update_meca(struct strategie* strat) {
}

// Cherche le prochain objectif
void strategie_fetch(struct strategie* strat) {
  int16_t robot_x,robot_y,robot_a;
  position_abs(&strat->pos, &robot_x, &robot_y, &robot_a);
  strat->verre = findClosest(&(strat->gm), robot_x, robot_y);
  strat->cur_objectif = &global_chemin;
  global_chemin.next = NULL;
  global_chemin.target.x = strat->verre->x;
  global_chemin.target.y = strat->verre->y;
}

void strategie_set_color(struct strategie* strat, enum cote side){
  // ReInitialisation de la position de départ
  strat->side = side;
  if(side == COTE_ROUGE) {
    position_begin_set(&strat->pos, -140, 100, 180);
  }
  else if(side == COTE_BLEU) {
    position_begin_set(&strat->pos, 140, 100, 0);
  }
  else {
    printf("ERREUR : dans l'initialisation du depart");
  } 

}
