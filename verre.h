#ifndef _VERRE_H_
#define _VERRE_H_
#include "tests.h"
#ifndef TEST
#include <aversive.h>
#endif

typedef struct {
	int16_t x;
	int16_t y;
} glass_t;

typedef struct {
	glass_t glass[12];
	uint8_t nb_glass_table;
	uint8_t nb_glass_robot;
} glass_manager;

//initialise l'ensemble des verres sur la table
void glassManagerInit(glass_manager* gm);
//prends un verre de la table et retourne le nombre de verre maintenant stocke
uint8_t takeGlass(glass_manager* gm, glass_t* gt);
//trouve le verre le plus proche de notre position actuelle
glass_t* findClosest(glass_manager* gm, int16_t x, int16_t y);
//supprime un verre de la liste, car celui ci n'est plus positionné correctement  ou a été déplacé
void removeGlass(glass_manager* gm, glass_t* gt);

#endif //_VERRE_H_

