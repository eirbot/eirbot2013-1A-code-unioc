

#include <aversive.h>
#include <wait.h>
#include <time.h>
#include <uart.h>
#include <scheduler.h>
#include <i2cm.h>
#include <adc.h>
#include <math.h>
#include "unioc_config.h"
#include "com.h"
#include "position_manager.h"
#include "asserv_manager.h"
#include "trajectory_manager.h"
#include "asserv_algo.h"

#include "gp2.h"

#include "strategie.h"

// Compilation conditionnelle
#include "strat_config.h"

#define PIN_TIRETTE (1 << 5)

#define ALIM_START (1)
#define LARGEUR_ROBOT 30.0
#define get_fdc 0x16

#define R_ENC 20  // Rapport de valeur entre les encodeurs internes et externes


// Prototypes //////////////////////////////////////////
void init_couleur_depart(void);
void initCom(void);
uint8_t mecaCom(uint8_t ordre);
int8_t get_FdCourse_avant(void);
int8_t get_FdCourse_arriere(void);
void rdsInit(void);
int getDistance(uint16_t code);
void adversary_detection(void);
uint8_t get_adversary_position(void); // attention, l'information "devant", "derriere" etc ne sont pas tout a fait fiable dans le sens ou la fonction renvoie "devant" si elle ne sait pas si c'est devant, derriere etc mais qu'elle sait qu'il y a quelque chose pas très loin (i.e plus de 2 TSOP allumés)
void detection_adversaire(void *);
void goto_d_stop_skating(trajectory_manager_t * traj, double d);
void goto_d_back_detection(trajectory_manager_t * traj, double d);

//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\//

///////////// Declaration de variables globales et enumeration //////////////////////////
enum{ROUGE = -1, PRESQUE_ROUGE = 0, BLEU = 1, PRESQUE_BLEU = 2}; // valeurs renvoyées par couleur(..), et utilisées aussi pour couleur_depart()
enum{ATTEND, CALAGE, SORTIE_SIMPLE, CHERCHE1, CHERCHE_DIAG, DECALE_DIAG, CHERCHE_COIN_BONUS_ADVERS, CHERCHE_PION_MILIEU, POSE_ZONE_SECURISEE_ADVERS, POSE_COIN_BONUS, CHERCHE_A_RECULON, POSE_A_RECULON_ZONESECURISEE, AVANT, ARRIERE, POSE}; // différents modes dans lesquels le robot peut se trouver
enum{POS_ADV_INCONNUE, POS_ADV_DEVANT, POS_ADV_DERRIERE, POS_ADV_GAUCHE, POS_ADV_DROITE}; // valeurs renvoyées par get_adversary_position(..)

trajectory_manager_t traj;

void clean_traj(void) {
  traj.current = traj.last;
}

int8_t mode_fonctionnement;
extern int8_t patinage;
int8_t I2C_DISPO = 1; //1 = dispo, 0 = en cours d'utilisation. Cette variable permet d'eviter les conflits lors d'une utilisation simultanee de l'i2c (surtout pour la fonction maj_gp2 dont on ne maitrise pas le moment d'execution)

/* Dire d'utiliser ou non les GP2 à l'avant (respectivement à l'arrière)
 * pour s'arrêter en cas de détection d'un obstacle */
int8_t AVANCE=1;
int8_t RECULE=0;
//int8_t Launch_bis=0;
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\//

// [1] => 1er en partant du bas // gp2 arriere
// [0] => 3e en partant du bas //gp2 droit 
// [2] => 4e en partant du bas //gp2 gauche
// [3] => 5e en partant du bas //gp2 milieu
// [4] => 2e en partant du bas // pas un adc
#define MUX_GP2_ARRIERE (ADC_REF_AVCC | MUX_ADC1) 
#define MUX_GP2_GAUCHE (ADC_REF_AVCC | MUX_ADC2)
#define MUX_GP2_DROITE (ADC_REF_AVCC | MUX_ADC0) 
#define MUX_GP2_AVANT (ADC_REF_AVCC | MUX_ADC3) 

// Init des GP2
struct gp2_cfg_t gp2[GP2_MAX];

//*** a supprimer ***//
//variable globale pour detection
//uint8_t trajectory_is_pause = 0;
//***
 uint16_t gp2_ga = 0;
 uint16_t gp2_dr = 0;
 uint16_t gp2_av = 0;
 uint16_t gp2_ar = 0;


 
uint8_t DETECTION_ACTIVATED = 1;
#define DIST_STOP 30

void detection(void *p){
 gp2_ga = gp2_get_dist(&gp2[GP2_GAUCHE],adc_get_value(MUX_GP2_GAUCHE));
 gp2_dr = gp2_get_dist(&gp2[GP2_DROITE],adc_get_value(MUX_GP2_DROITE));	  
 gp2_av = gp2_get_dist(&gp2[GP2_AVANT],adc_get_value(MUX_GP2_AVANT));
 gp2_ar = gp2_get_dist(&gp2[GP2_ARRIERE],adc_get_value(MUX_GP2_ARRIERE));
 gp2_av = 150;
 //if(DETECTION_ACTIVATED) {
   if (((gp2_ga < DIST_STOP || gp2_dr < DIST_STOP || gp2_av < DIST_STOP) && AVANCE)|| (gp2_ar < DIST_STOP && RECULE))
     {
       if (!trajectory_is_paused(&traj)) {
	 printf("Traj paused\n");
	 trajectory_pause(&traj);
       }
     }
   else{
     printf("Auncun Objet detecte \n");
     if (trajectory_is_paused(&traj))
       trajectory_resume(&traj);
   }
   // }
}
void skate(void *p) {
  static int16_t last_x=0, last_y=0, last_a=0;
  int16_t x, y, a;
  position_abs(&traj, &x, &y, &a);
  if(x == last_x && y == last_y) {
    //trajectory_goto_arel(&traj, NOW, 90);
     trajectory_goto_d(&traj, NOW, -6);
 }
}
int main(void)
{
	//Declaration des variables//////////
	position_manager_t pos; 
	asserv_manager_t asserv;
	struct strategie strat;

	int i = 0;

	/////////////////////////////////////

	//Initialisations////////////////////
	uart_init();
	time_init(128);
	fdevopen(uart0_send,NULL, 0);
	/***** tip -s 38400 -l /dev/ttyS1 *****/

	//adc_init();
	DDRE |= 0xf0;

	//initialisation du fpga
	sbi(XMCRA,SRW11);
	sbi(XMCRA,SRW00);
	sbi(MCUCR,SRE);
	wait_ms(1000);

	//on reset le fpga
	U_RESET = 0x42;
	sbi(PORTB,0);
	wait_ms(100);

	//on relache le reset
	cbi(PORTB,0);
	sbi(PORTE,3);
	wait_ms(100);
 
	//init des interruptions (detection)
	scheduler_init();
	scheduler_add_periodical_event(detection, NULL, 50000/SCHEDULER_UNIT);
	//scheduler_add_periodical_event(skate, NULL,     50000000/SCHEDULER_UNIT);

	
#ifndef MAIN
	position_init(&pos);
	asserv_init(&asserv,&pos);
	trajectory_init(&traj,&pos,&asserv);
	i2cm_init();
	sei();  // autorise les interruptions
#endif

	//// Init ADC
	adc_init();

	//// GP2 Avant Gauche
	gp2_init(&gp2[GP2_GAUCHE]);

	gp2_add_point(&gp2[GP2_GAUCHE], 150,  500);
	gp2_add_point(&gp2[GP2_GAUCHE], 120,  600);
	gp2_add_point(&gp2[GP2_GAUCHE], 100,  700);
	gp2_add_point(&gp2[GP2_GAUCHE],  80,  850);
	gp2_add_point(&gp2[GP2_GAUCHE],  70,  900);
	gp2_add_point(&gp2[GP2_GAUCHE],  60, 1050);
	gp2_add_point(&gp2[GP2_GAUCHE],  50, 1250);
	gp2_add_point(&gp2[GP2_GAUCHE],  40, 1500);
	gp2_add_point(&gp2[GP2_GAUCHE],  30, 2000);
	gp2_add_point(&gp2[GP2_GAUCHE],  20, 2500);

	// GP2 Avant Droite
	gp2_init(&gp2[GP2_DROITE]);

	gp2_add_point(&gp2[GP2_DROITE], 150,  500);
	gp2_add_point(&gp2[GP2_DROITE], 120,  600);
	gp2_add_point(&gp2[GP2_DROITE], 100,  700);
	gp2_add_point(&gp2[GP2_DROITE],  80,  850);
	gp2_add_point(&gp2[GP2_DROITE],  70,  900);
	gp2_add_point(&gp2[GP2_DROITE],  60, 1050);
	gp2_add_point(&gp2[GP2_DROITE],  50, 1250);
	gp2_add_point(&gp2[GP2_DROITE],  40, 1500);
	gp2_add_point(&gp2[GP2_DROITE],  30, 2000);
	gp2_add_point(&gp2[GP2_DROITE],  20, 2500);

	// GP2 Avant Millieu
	gp2_init(&gp2[GP2_AVANT]);

	gp2_add_point(&gp2[GP2_AVANT],  80, 400);
	gp2_add_point(&gp2[GP2_AVANT],  70, 450);
	gp2_add_point(&gp2[GP2_AVANT],  60, 550);
	gp2_add_point(&gp2[GP2_AVANT],  50, 625);
	gp2_add_point(&gp2[GP2_AVANT],  40, 760);
	gp2_add_point(&gp2[GP2_AVANT],  30, 975);
	gp2_add_point(&gp2[GP2_AVANT],  20, 1400);
	gp2_add_point(&gp2[GP2_AVANT],  10, 2500);

	// GP2 Arriere
	//2y0a02f
	gp2_init(&gp2[GP2_ARRIERE]);

	gp2_add_point(&gp2[GP2_ARRIERE],  80, 400);
	gp2_add_point(&gp2[GP2_ARRIERE],  70, 450);
	gp2_add_point(&gp2[GP2_ARRIERE],  60, 550);
	gp2_add_point(&gp2[GP2_ARRIERE],  50, 625);
	gp2_add_point(&gp2[GP2_ARRIERE],  40, 760);
	gp2_add_point(&gp2[GP2_ARRIERE],  30, 975);
	gp2_add_point(&gp2[GP2_ARRIERE],  20, 1400);
	gp2_add_point(&gp2[GP2_ARRIERE],  10, 2500);
	

	// Init de la position
	//position_begin_set(&pos, -140, 100, 0);
	position_begin_set(&pos, 140, 100, 180);




	// STRATEGIE PRINCIPALE
#ifdef MAIN
	
	int16_t x,y,a;

	strategie_init(&strat, COTE_BLEU);
	wait_ms(100);
	//strategie_fetch(&strat);
	//position_begin_set(&(strat.pos), 140, 100, 180);
	asserv_init(&asserv,&(strat.pos));
	wait_ms(100);
	trajectory_init(&traj,&(strat.pos),&asserv);
	i2cm_init(); // 
	sei();  // autorise les interruptions
	//trajectory_goto_d(&traj, NOW, 2);
	//while(!trajectory_is_ended(&traj));
	wait_ms(200);//laisser au fpga le temps de repondre..
	position_abs(&(strat.pos), &x, &y, &a);
	printf("je suis ici %d %d %d\n", x, y, a);
	strategie_fetch(&strat);
	
	
#endif//MAIN


#ifdef ALIM_START
	  // Gestion de la tirette
	  DDRE &= ~PIN_TIRETTE; // En lecture
	  while(!(PINE & PIN_TIRETTE)) {
	    wait_ms(100);
	    printf("%d\n", PINE);
	  }
#endif

#ifdef MAIN
	  enum cote side = COTE_BLEU;
	  DDRE &= ~(1<<7);
	  if(PINE & (1<<7)) {
	    side = COTE_ROUGE;
	  }
	  else {
	    side = COTE_BLEU;
	  }

	  strategie_set_color(&strat, side);

	  wait_ms(100);
strategie_fetch(&strat);
	  mecaCom(MECA_START);
	    
	mecaCom(MONTER); //test
#endif//MAIN


#ifdef TEST_POS
	  position_begin_set(&pos, -140, 100, 180);
#endif
	  uint8_t skate = 0;
	int8_t comptour=0;

	while(1) {
#ifdef TOUPIE
	  trajectory_goto_arel(&traj, END, 90);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);
#endif //TOUPIE
#ifdef MAIN	



	  //strategie_update_gp2(&strat, gp2_ga, gp2_dr, gp2_av, 0);
	  
/* if(is_skating()) {
	    skate++;
	    goto_d_back_detection(&traj, -3);
	    trajectory_goto_arel(&traj,NOW, 90);
	    if(skate == 10) {
	      DETECTION_ACTIVATED = 0;
trajectory_pause(&traj);
while(1);}
	    
}*/

	  //strategie_update_meca(&strat);
	  if(comptour >= 7) {
	    trajectory_pause(&traj);
	    DETECTION_ACTIVATED = 0;
	    while(1);
	  }

	  if(strat.gm.nb_glass_robot < 1) { 
	  if(!strategie_arrived(&strat)) {
		
		//strategie_where(&strat, &x, &y, &a);
		if(trajectory_is_ended(&traj)) {
		  strategie_where(&strat, &x, &y, &a);
		  trajectory_goto_pos_wa(&traj,END, x, y);
		  printf("heading %d %d %d\n", x, y , a);
		  printf("cur pos : %d, %d (%d)", strat.pos.x, strat.pos.y, strat.pos.angle);
		}
	  	//while(!trajectory_is_ended(&traj));
		
	  }
	  else {


		printf("Nouveau verre\n");
		mecaCom(SORTIR_RABBATEUR);
		mecaCom(RENTRER_RABBATEUR);
		mecaCom(DESCENDRE_SUR_VERRE);
		mecaCom(FERMER_GAUCHE);
		//mode triche
		if(strat.verre != NULL)
		takeGlass(&(strat.gm), strat.verre);
		//fin du mode triche
	  	if(strat.gm.nb_glass_robot < 4) { 
			mecaCom(MONTER);
		}
		strategie_fetch(&strat);}
		  
	  }
	  else {
	    if(strat.side == COTE_ROUGE)
	      trajectory_goto_pos_wa(&traj, END, -125, 100+0*comptour);
	    if(strat.side == COTE_BLEU)
	    trajectory_goto_pos_wa(&traj, END, 125, 100+01*comptour);
	    while(!trajectory_is_ended(&traj));
		mecaCom(DEPOSER);
	    goto_d_back_detection(&traj, -30);
	    while(!trajectory_is_ended(&traj));
	    //trajectory_goto_pos_wa(&traj, END, 0, 100);
	    strat.gm.nb_glass_robot=0;//while(1);
            comptour ++;
	  }
	  /*
       if (!trajectory_is_paused(&traj)) {
	 printf("Traj paused\n");
	 trajectory_pause(&traj);
	 wait_ms(1000);
       }
      else{
     
     
       trajectory_resume(&traj);
       }
       /**/

	  

#endif//MAIN

#ifdef QUADRAMP

	  quadramp_set_2nd_order_vars(&asserv.qramp_distance, 2, 2 );
	  quadramp_set_1st_order_vars(&asserv.qramp_distance, 50, 50);

	  trajectory_goto_d(&traj, END, 40);
	  while(!trajectory_is_ended(&traj));

	  quadramp_set_2nd_order_vars(&asserv.qramp_distance, 8, 8 );
	  quadramp_set_1st_order_vars(&asserv.qramp_distance, 150, 150);
	 
	  trajectory_goto_d(&traj, END, 40);
	  while(!trajectory_is_ended(&traj));
	 

#endif//QUADRAMP

#ifdef MATCH // FAIIIILLL
	  // Début, mettre la droite du robot en face des verres les plus à droite. 

	  mecaCom(ATTRAPER_VERRES);

	  //Démarrage en douceur
	  quadramp_set_2nd_order_vars(&asserv.qramp_distance,3,3);
	  quadramp_set_1st_order_vars(&asserv.qramp_distance,50,50);

	  // On avance jusqu'au premier verre
	  //// GESTION DE LA VITESSE POUR ARRIVER VITE
	  trajectory_goto_d(&traj, END, 20);
	  while(!trajectory_is_ended(&traj));

	  quadramp_set_2nd_order_vars(&asserv.qramp_distance, 4,4);
	  quadramp_set_1st_order_vars(&asserv.qramp_distance, 95,95);	  
	  trajectory_goto_d(&traj, END, 55);
	  while(!trajectory_is_ended(&traj));

	  quadramp_set_2nd_order_vars(&asserv.qramp_distance, 5,5);
	  quadramp_set_1st_order_vars(&asserv.qramp_distance, 80,80);

	  // Distance parcourue : 75

	  //mecaCom(FERMER_DROITE);

	  // Deviation pour mettre le 2e verre à gauche
	  trajectory_goto_arel(&traj, END, -8);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  // On avance jusqu'au deuxième verre
	  trajectory_goto_d(&traj, END, 40);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  //mecaCom(FERMER_GAUCHE);

	  // Déviation dans l'autre sens pour se remettre droit
	  trajectory_goto_arel(&traj, END, 8);
	  while(!trajectory_is_ended(&traj));
	  //wait_ms(100);

	  // On tourne à gauche
	  trajectory_goto_arel(&traj, END, 90);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  // On se décale sur l'autre colone
	  trajectory_goto_d(&traj, END, 28);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  // On retourne vers notre camp
	  trajectory_goto_arel(&traj, END, 90);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  // On avance vers notre camp
	  //// Partie 1 : On avance avec la detection
	  trajectory_goto_d(&traj, END, 65);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(10);
	  //// Partie 2 : On avance sans la detections pour les bord du plateau
	  DETECTION_ACTIVATED = 0;
	  trajectory_goto_d(&traj, END, 35);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);
	  //// On REACTIVE les GP2 !!
	  DETECTION_ACTIVATED = 1;
	  AVANCE = 0;
	  //// Distance parcourue : 100 cm !

	  mecaCom(DEPOSER);

	  // On reculepour laisser les verres DANS NOTRE CAMP DE PREFERRENCE
	  RECULE = 1;
	  trajectory_goto_d(&traj, END, -25);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);
	  RECULE = 0;
	  
	  mecaCom(ATTRAPER_VERRES);


	  // On se retourne
	  trajectory_goto_arel(&traj, END, 180);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);
	  AVANCE = 1;

	  // On repart
	  trajectory_goto_d(&traj, END, 95);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  // On tourne à gauche vers la troisième colone
	  trajectory_goto_arel(&traj, END, 90);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  // On avance pour se placer vers la troisième colone
	  trajectory_goto_d(&traj, END, 24);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  // On trourne vers notre camp
	  trajectory_goto_arel(&traj, END, 90);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  // On avance vers notre camp
	  trajectory_goto_d(&traj, END, 75);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(10);
	  
	  DETECTION_ACTIVATED = 0;
	  trajectory_goto_d(&traj, END, 35);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  DETECTION_ACTIVATED = 1;
	  AVANCE = 0;
	  // Distance parcourue : 110 cm !

	  mecaCom(DEPOSER);

	  // On recule pour laisser les verres dans notre camp de préferrence
	  RECULE = 1;
	  trajectory_goto_d(&traj, END, -25);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);
	  RECULE = 0;

	  mecaCom(ATTRAPER_VERRES);


	  // On repart
	  trajectory_goto_arel(&traj, END, 180);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);
	  AVANCE = 1;

	  //// TEST D'ETAPE
	  ////// On est au milieu de la table
	  for(i = 0 ; i < 4 ; i++) {
	    trajectory_goto_arel(&traj, END, -90);
	    while(!trajectory_is_ended(&traj));
	    wait_ms(100);
	  }
	  
	  /////

	  trajectory_goto_d(&traj, END, 130);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  trajectory_goto_arel(&traj, END, -8);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  trajectory_goto_d(&traj, END, 40);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  trajectory_goto_arel(&traj, END, 8);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);


	  mecaCom(FERMER_DROITE);
	  mecaCom(FERMER_GAUCHE);

	  trajectory_goto_arel(&traj, END, -90);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  trajectory_goto_d(&traj, END, 20);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  trajectory_goto_arel(&traj, END, -90);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  trajectory_goto_d(&traj, END, 190);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  mecaCom(DEPOSER);

	  trajectory_goto_d(&traj, END, -25);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  mecaCom(ATTRAPER_VERRES);


	  trajectory_goto_arel(&traj, END, 180);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  trajectory_goto_d(&traj, END, 170);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  trajectory_goto_arel(&traj, END, -90);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  trajectory_goto_d(&traj, END, 25);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  trajectory_goto_arel(&traj, END, -90);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  trajectory_goto_d(&traj, END, 200);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  mecaCom(DEPOSER);

	  trajectory_goto_d(&traj, END, -25);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  mecaCom(ATTRAPER_VERRES);

	  trajectory_goto_arel(&traj, END, 180);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  trajectory_goto_d(&traj, END, 30);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);
#endif

#ifdef MATCH_2
	  // Début, mettre la droite du robot en face des verres les plus à droite. 

	  mecaCom(ATTRAPER_VERRES);

	  //Démarrage en douceur
	  quadramp_set_2nd_order_vars(&asserv.qramp_distance,3,3);
	  quadramp_set_1st_order_vars(&asserv.qramp_distance,50,50);

	  // On avance jusqu'au premier verre
	  //// GESTION DE LA VITESSE POUR ARRIVER VITE
	  trajectory_goto_d(&traj, END, 20);
	  while(!trajectory_is_ended(&traj));

	  quadramp_set_2nd_order_vars(&asserv.qramp_distance, 4,4);
	  quadramp_set_1st_order_vars(&asserv.qramp_distance, 95,95);	  
	  trajectory_goto_d(&traj, END, 55);
	  while(!trajectory_is_ended(&traj));

	  quadramp_set_2nd_order_vars(&asserv.qramp_distance, 5,5);
	  quadramp_set_1st_order_vars(&asserv.qramp_distance, 80,80);

	  // Distance parcourue : 75

	  //mecaCom(FERMER_DROITE);

	  // Deviation pour mettre le 2e verre à gauche
	  trajectory_goto_arel(&traj, END, -8);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  // On avance jusqu'au deuxième verre
	  trajectory_goto_d(&traj, END, 40);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  //mecaCom(FERMER_GAUCHE);

	  // Déviation dans l'autre sens pour se remettre droit
	  trajectory_goto_arel(&traj, END, 8);
	  while(!trajectory_is_ended(&traj));
	  //wait_ms(100);

	  // On tourne à gauche
	  trajectory_goto_arel(&traj, END, 90);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  // On se décale sur l'autre colone
	  trajectory_goto_d(&traj, END, 28);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  // On retourne vers notre camp
	  trajectory_goto_arel(&traj, END, 90);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  mecaCom(MONTER_REMPLI);
	  wait_ms(2000);

	  // On avance vers notre camp
	  //// Partie 1 : On avance avec la detection
	  trajectory_goto_d(&traj, END, 65);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(10);
	  //// Partie 2 : On avance sans la detections pour les bord du plateau
	  DETECTION_ACTIVATED = 0;
	  trajectory_goto_d(&traj, END, 35);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);
	  //// On REACTIVE les GP2 !!
	  DETECTION_ACTIVATED = 1;
	  AVANCE = 0;
	  //// Distance parcourue : 100 cm !

	  mecaCom(DESCENDRE_SECURE);
	  wait_ms(2000);
	  mecaCom(MONTER_REMPLI);
	  wait_ms(2000);

	  // On reculepour laisser les verres DANS NOTRE CAMP DE PREFERRENCE
	  RECULE = 1;
	  trajectory_goto_d(&traj, END, -25);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);
	  RECULE = 0;
	  
	  mecaCom(ATTRAPER_VERRES);


	  // On se retourne
	  trajectory_goto_arel(&traj, END, 180);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);
	  AVANCE = 1;

	  // On repart
	  trajectory_goto_d(&traj, END, 95);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  // On tourne à gauche vers la troisième colone
	  trajectory_goto_arel(&traj, END, 90);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  // On avance pour se placer vers la troisième colone
	  trajectory_goto_d(&traj, END, 24);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  // On trourne vers notre camp
	  trajectory_goto_arel(&traj, END, 90);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  mecaCom(MONTER_REMPLI);
	  wait_ms(2000);

	  // On avance vers notre camp
	  trajectory_goto_d(&traj, END, 75);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(10);
	  
	  DETECTION_ACTIVATED = 0;
	  trajectory_goto_d(&traj, END, 35);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  DETECTION_ACTIVATED = 1;
	  AVANCE = 0;
	  // Distance parcourue : 110 cm !

	  mecaCom(DESCENDRE_SECURE);
	  wait_ms(2000);
	  mecaCom(MONTER_REMPLI);
	  wait_ms(2000);


	  // On recule pour laisser les verres dans notre camp de préferrence
	  RECULE = 1;
	  trajectory_goto_d(&traj, END, -25);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);
	  RECULE = 0;

	  mecaCom(ATTRAPER_VERRES);


	  // On repart
	  trajectory_goto_arel(&traj, END, 180);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);
	  AVANCE = 1;

	  //// TEST D'ETAPE
	  ////// On est au milieu de la table
	  for(i = 0 ; i < 4 ; i++) {
	    trajectory_goto_arel(&traj, END, -90);
	    while(!trajectory_is_ended(&traj));
	    wait_ms(100);
	  }
	  
	  /////

	  trajectory_goto_d(&traj, END, 130);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  trajectory_goto_arel(&traj, END, -8);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  trajectory_goto_d(&traj, END, 40);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  trajectory_goto_arel(&traj, END, 8);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);


	  mecaCom(FERMER_DROITE);
	  mecaCom(FERMER_GAUCHE);

	  trajectory_goto_arel(&traj, END, -90);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  trajectory_goto_d(&traj, END, 20);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  trajectory_goto_arel(&traj, END, -90);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  trajectory_goto_d(&traj, END, 190);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  mecaCom(DEPOSER);

	  trajectory_goto_d(&traj, END, -25);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  mecaCom(ATTRAPER_VERRES);


	  trajectory_goto_arel(&traj, END, 180);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  trajectory_goto_d(&traj, END, 170);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  trajectory_goto_arel(&traj, END, -90);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  trajectory_goto_d(&traj, END, 25);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  trajectory_goto_arel(&traj, END, -90);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  trajectory_goto_d(&traj, END, 200);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  mecaCom(DEPOSER);

	  trajectory_goto_d(&traj, END, -25);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  mecaCom(ATTRAPER_VERRES);

	  trajectory_goto_arel(&traj, END, 180);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);

	  trajectory_goto_d(&traj, END, 30);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);
#endif//MATCH_2


#ifdef TEST_POS

	  trajectory_goto_pos(&traj,END, -120, 90,0);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(1000);
	  trajectory_goto_pos(&traj,END, -100, 100,0);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(1000);
	  trajectory_goto_pos(&traj,END, -140, 100,180);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(1000);	  
#endif
		
//Test i2c
#ifdef TEST_I2C
	  /*
	  uint8_t slave_addr = 2;
	  uint8_t size_data = 1;
	  uint8_t ordre = DEUX_PILES;
	  i2cm_send(slave_addr, size_data, &ordre);
	  //**on se déplace jusqu'à un verre** 
	  //on demande à la carte meca si bien reçu (1 = reçu, 0 = non..)
	  ordre = QUESTION_VERRE_RECUP;
	  i2cm_send(slave_addr, size_data, &ordre);
	  int bool_verre_attrape = i2cm_rcv_single(slave_addr);
	  printf("Verre recu ? : %d \n", bool_verre_attrape);
	  //combien de verres en tout?
	  ordre = COMBIEN_VERRES;
	  i2cm_send(slave_addr, size_data, &ordre);
	  uint8_t nbr_verres;
	  uint8_t tmp;
	  ordre = mecaBusy;
	  while (ordre != mecaReady){
	    i2cm_rcv(slave_addr, size_data, &nbr_verres);
	    if (ordre != mecaReady)
	      tmp = ordre;
	    wait_ms(50);
	  }
	  nbr_verres = tmp;
	  printf("Combien de verres en tout ? : %d \n", nbr_verres);
	  */
	  mecaCom(DEUX_PILES);
	  while(1){
	    wait_ms(1000);
	    printf("\nVerre reçu : %d\n",mecaCom(QUESTION_VERRE_RECUP));
	    printf("Nb de verres : %d\n",mecaCom(COMBIEN_VERRES));
	  }
	  


#endif

// Test des GP2
#ifdef TEST_GP2
	  uint16_t gp2_ga = adc_get_value(MUX_GP2_GAUCHE);
	  printf("Gauche : %d", gp2_get_dist(&gp2[GP2_GAUCHE], gp2_ga));

	  uint16_t gp2_dr = adc_get_value(MUX_GP2_DROITE);
	  printf("Droite : %d", gp2_get_dist(&gp2[GP2_DROITE], gp2_dr));

	  uint16_t gp2_av = adc_get_value(MUX_GP2_AVANT);
	  printf("Milieu : %d", gp2_get_dist(&gp2[GP2_AVANT], gp2_av));
	  
	  uint16_t gp2_ar = adc_get_value(MUX_GP2_ARRIERE);
	  printf("Arriere : %d\n", gp2_get_dist(&gp2[GP2_ARRIERE], gp2_ar));
	  
	  wait_ms(300);
#endif

//HOMOLOGATION*****************************************************
#ifdef HOMOLOGATION
	  printf("Starting movement\n");
	  trajectory_goto_d(&traj, END, 140);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);
	  trajectory_goto_arel(&traj, END, 90);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);
	  trajectory_goto_d(&traj, END, 40);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);
	  trajectory_goto_arel(&traj, END, 90);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);
	  trajectory_goto_d(&traj, END, 125);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);
	  goto_d_back_detection(&traj, -10);
	  //trajectory_goto_d(&traj, END, -20);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);
	  while(1);
#endif
	  
// code de presentation dans la rue : 23/04/13
#ifdef TEST_PRESENTATION_RUE
	  trajectory_goto_d(&traj, END, 60);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(500);
	  trajectory_goto_d(&traj, END, 30);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(500);
	  trajectory_goto_arel(&traj, END, 90);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(250);
	  trajectory_goto_d(&traj, END, 55);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(500);
	  trajectory_goto_arel(&traj, END, 90);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(250);
	  trajectory_goto_d(&traj, END, 30);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(500);
	  trajectory_goto_arel(&traj, END, 45);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(250);
	  trajectory_goto_d(&traj, END, 80);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(500);
	  trajectory_goto_arel(&traj, END, 45);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(250);
	  trajectory_goto_d(&traj, END, 55);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(500);
	  trajectory_goto_d(&traj, END, 5);
	  while(!trajectory_is_ended(&traj));
#endif

// Premier Test de trajectory_goto_pos
#ifdef TEST_TRAJECTORY_POS
	  trajectory_goto_pos(&traj, END, -110 , 130, 90);
	  while(!trajectory_is_ended(&traj));
	  trajectory_goto_pos(&traj, END, -120, 170, 0);
	  while(!trajectory_is_ended(&traj));
	  trajectory_goto_pos(&traj, END, -100, 160, 0);
	  while(!trajectory_is_ended(&traj));
	  trajectory_goto_pos(&traj, END, -120, 110, 0);
	  while(!trajectory_is_ended(&traj));
	  trajectory_goto_pos(&traj, END, -110, 90, 0);
	  while(!trajectory_is_ended(&traj));
	  trajectory_goto_pos(&traj, END, -140, 100, 0);
	  while(!trajectory_is_ended(&traj));
#endif

// Permet d'ajuster la position en angle
#ifdef TEST_POS_ANGLE
	  for(i = 0 ; i < (4 * 20) ; i++) {
	    trajectory_goto_arel(&traj, END, 90);
	    while(!trajectory_is_ended(&traj));
	    printf("%ld %ld %ld %ld\n", U_ENC0, U_ENC1, U_ENC2, U_ENC3);
	  }
#endif

// Permet d'ajuster la position en distance
#ifdef TEST_POS_DIST	
	  printf("%ld %ld %ld %ld\n", U_ENC0, U_ENC1, U_ENC2, U_ENC3);
	  trajectory_goto_d(&traj, END, 50);
	  while(!trajectory_is_ended(&traj));
	  printf("%ld %ld %ld %ld\n", U_ENC0, U_ENC1, U_ENC2, U_ENC3);
	  while(1);
#endif
	  /**/
	  /*
	  trajectory_goto_a(&traj, END, -90);
	  while(!trajectory_is_ended(&traj));
	  trajectory_goto_d(&traj, END, 50);
	  while(!trajectory_is_ended(&traj));
	  trajectory_goto_a(&traj, END, -90);
	  while(!trajectory_is_ended(&traj));
	  /**/
	  /*
	  trajectory_goto_d(&traj, END, 100);
	  while(!trajectory_is_ended(&traj));
	  trajectory_goto_a(&traj, END, -90);
	  while(!trajectory_is_ended(&traj));
	  trajectory_goto_d(&traj, END, 100);
	  while(!trajectory_is_ended(&traj));
	  trajectory_goto_a(&traj, END, 230);
	  while(!trajectory_is_ended(&traj));
	  /**/
	  //while(1);
	}
}

/**
 * Renvoie 1 si le robot est bloqué (patine)
 * (i.e. les valeurs des encodeurs internes
 * et externes ne correspondent pas) 
 **/

int8_t is_skating(void)  
{
	int8_t flag = 0; 
	static int32_t uenc0old = 0, uenc2old = 0;
	static int32_t uenc1old = 0, uenc3old = 0;

	if ((R_ENC * (double)(ABS(U_ENC0-uenc0old)) < (double)(ABS(U_ENC1-uenc1old))) ||
			(R_ENC * (double)(ABS(U_ENC2-uenc2old)) < (double)(ABS(U_ENC3-uenc3old))))
		flag = 1;

	uenc0old = U_ENC0;
	uenc1old = U_ENC1;
	uenc2old = U_ENC2;
	uenc3old = U_ENC3;

	return flag;
}
int8_t is_skating2(void)  
{
	int8_t flag = 0; 
	
	if ((100 * (double)(ABS(U_ENC0)) < (double)(ABS(U_ENC1))) ||
			(100 * (double)(ABS(U_ENC2)) < (double)(ABS(U_ENC3))))
		flag = 1;

	
	return flag;
}

/**
 * Fait se déplacer le robot d'une distance de d cm
 * et s'arrête lorsque le robot rentre dans un obstacle 
*/
void goto_d_stop_skating(trajectory_manager_t * traj, double d){ 
	trajectory_goto_d(traj, END, d);
	wait_ms(500);
	while(!trajectory_is_ended(traj)){
		wait_ms(100);
		if(is_skating()){
			trajectory_pause(traj);
			trajectory_removeStep(traj);
		}
	}
}

/**
 * Fait translater le robot avec les GP2 activés à l'arrière et désactivés à l'avant.
 * Puis désactive les GP2 à l'arrière et les active à l'avant.
 *
 * @param d		Le nombre de cm de déplacement du robot
 */
void goto_d_back_detection(trajectory_manager_t * traj, double d){
	AVANCE=0;
	RECULE=1;
	trajectory_goto_d(traj, END, d);//sortie cale
	while(!trajectory_is_ended(traj));
	AVANCE=1;
	RECULE=0;
}
uint8_t mecaCom(uint8_t commande) {//BA2
  return 0;
  I2C_DISPO = 0;
	uint8_t result = -1;
	i2cm_send(mecaADDR,1,&commande);
	wait_ms(100);
	uint8_t reponse = mecaBusy;
	while(reponse != mecaReady) {
		i2cm_rcv(mecaADDR,1,&reponse);
		printf("Commande %d, reponse %d\n", commande, reponse);
		if(reponse != mecaReady)
			result = reponse; // C'est ici que l'on recoit effectivement la reponse de la meca
		wait_ms(50);
	}
	I2C_DISPO = 1;
	return result;
}
void init_couleur_depart(void)
{
	if(PINE&0x80)
	{
		traj_set_couleur_depart(-1); //si on est en ROUGE
	}
	else
	{
		traj_set_couleur_depart(1); //si on est en BLEU
	}
}
