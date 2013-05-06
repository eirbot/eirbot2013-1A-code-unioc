/////////////////////////////////////////////////////////////////////////////////
//            UNIOC
// 
// Derniere modification:
// Matt, le 20/12/2011 13h
// 
// Commentaire:
// 
//
// A faire:
//valeur gp2: a 30 cm :264
//a 50 cm:29
//a 1m : 303
// Tout
//
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\


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
int8_t mode_fonctionnement;
extern int8_t patinage;
int8_t I2C_DISPO = 1; //1 = dispo, 0 = en cours d'utilisation. Cette variable permet d'eviter les conflits lors d'une utilisation simultanee de l'i2c (surtout pour la fonction maj_gp2 dont on ne maitrise pas le moment d'execution)

/* Dire d'utiliser ou non les GP2 à l'avant (respectivement à l'arrière)
 * pour s'arrêter en cas de détection d'un obstacle */
int8_t AVANCE=1;
int8_t RECULE=0;
//int8_t Launch_bis=0;
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\//

//donner des ordres à la carte meca
#define DEPOSER 0
#define DEUX_PILES 1
#define QUESTION_VERRE_RECUP 2
#define COMBIEN_VERRES 3

#define MUX_GP2_GAUCHE (ADC_REF_AVCC | MUX_ADC1) // 1er en partant du bas
#define MUX_GP2_DROITE (ADC_REF_AVCC | MUX_ADC0) // 3e en partant du bas
#define MUX_GP2_AVANT (ADC_REF_AVCC | MUX_ADC2) // 4e en partant du bas
// [3] => 5e en partant du bas
// [4] => 2e en partant du bas


// Init des GP2
struct gp2_cfg_t gp2[GP2_MAX];



//*** a supprimer ***//
//variable globale pour detection
//uint8_t trajectory_is_pause = 0;
//***

void detection(void *p){
  uint16_t gp2_ga = gp2_get_dist(&gp2[GP2_GAUCHE],adc_get_value(MUX_GP2_GAUCHE));
  uint16_t gp2_dr = gp2_get_dist(&gp2[GP2_DROITE],adc_get_value(MUX_GP2_DROITE));	  
  uint16_t gp2_av = gp2_get_dist(&gp2[GP2_AVANT],adc_get_value(MUX_GP2_AVANT));
  if (gp2_ga < 30 || gp2_dr < 30 || gp2_av < 30){
    if (!trajectory_is_paused(&traj)) {
	    printf("Traj not paused\n");
      trajectory_pause(&traj);
    }
  }else{
    printf("Auncun Objet detecte \n");
    if (trajectory_is_paused(&traj))
      trajectory_resume(&traj);
  }
}

int main(void)
{
	//Declaration des variables//////////
	position_manager_t pos; 
	asserv_manager_t asserv;
	

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


	position_init(&pos);
	asserv_init(&asserv,&pos);
	trajectory_init(&traj,&pos,&asserv);
	i2cm_init();
	sei();  // autorise les interruptions


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
	// Init de la position
	position_begin_set(&pos, -140, 100, 0);
	
	/////////////////////
	//CODE STRATEGIE   //
	////////////////////
	
	while(1){
	  /*
	  //Test i2c
	  uint8_t slave_addr = 2;
	  uint8_t size_data = 8;
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

	  /*
	  // Test GP2
	  uint16_t gp2_ga = adc_get_value(MUX_GP2_GAUCHE);
	  printf("Gauche : %d\n", gp2_get_dist(&gp2[GP2_DROITE], gp2_ga));

	  uint16_t gp2_dr = adc_get_value(MUX_GP2_DROITE);
	  printf("Droite : %d\n", gp2_get_dist(&gp2[GP2_DROITE], gp2_dr));

	  uint16_t gp2_av = adc_get_value(MUX_GP2_AVANT);
	  printf("Milieu : %d\n", gp2_get_dist(&gp2[GP2_AVANT], gp2_av));
	  
	  wait_ms(300);
	  */

	  //HOMOLOGATION N°1******************************************************
	  printf("Starting movement\n");
		trajectory_goto_d(&traj, END, 30);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);
	  trajectory_goto_arel(&traj, END, 90);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);
	  trajectory_goto_d(&traj, END, 30);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);
	  trajectory_goto_arel(&traj, END, 90);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);
	  trajectory_goto_d(&traj, END, 30);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);
	  trajectory_goto_arel(&traj, END, 90);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);
	  trajectory_goto_d(&traj, END, 30);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);
	  trajectory_goto_arel(&traj, END, 90);
	  while(!trajectory_is_ended(&traj));
	  wait_ms(100);
	  

	  




	  
	/*  
	// code de presentation dans la rue : 23/04/13
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
	/**/
	  /*
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
	  /**/
	  /*
	  for(i = 0 ; i < (4 * 20) ; i++) {
	    trajectory_goto_arel(&traj, END, 90);
	    while(!trajectory_is_ended(&traj));
	    printf("%ld %ld %ld %ld\n", U_ENC0, U_ENC1, U_ENC2, U_ENC3);
	  }

	  /**/
	  /*
	  trajectory_goto_d(&traj, END, 100);
	  while(!trajectory_is_ended(&traj));
	  printf("%ld %ld %ld %ld\n", U_ENC0, U_ENC1, U_ENC2, U_ENC3);
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
uint8_t mecaCom(uint8_t ordre) {//BA2
	I2C_DISPO = 0;
	uint8_t result = -1;
	i2cm_send(mecaADDR,1,&ordre);
	wait_ms(100);
	ordre = mecaBusy;
	while(ordre != mecaReady) {
		i2cm_rcv(mecaADDR,1,&ordre);
		if(ordre != mecaReady)
			result = ordre; // C'est ici que l'on recoit effectivement la reponse de la meca
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
