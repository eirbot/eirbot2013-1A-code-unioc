#ifndef COMMECA
#define COMMECA

//***communication meca***
#define mecaADDR 2

#define mecaBusy 101
#define mecaReady 102
#define DEUX_PILES 103
#define DEPOSER 104
#define QUESTION_VERRE_RECUP 105
#define COMBIEN_VERRES 106
#define ATTRAPER_VERRES 107
#define FERMER_GAUCHE 108
#define FERMER_DROITE 109
#define MONTER 110
#define DESCENDRE 111
#define MONTER_REMPLI 112
#define DESCENDRE_SECURE 113
#define MECA_START 114
#define SORTIR_RABBATEUR 115
#define RENTRER_RABBATEUR 116
#define DESCENDRE_SUR_VERRE 117

//les fonctions
// /!\ne pas associer des actions à 0x01 et 0x02 car c'est les même valeurs que mecaReady et mecaBusy et ça peut poser des problèmes un peu compliqué à expliquer
#define actFermerAvant 0x03
#define actOuvrirAvant 0x04
#define actRangerAvant 0x05
#define actFermerArriere 0x06
#define actOuvrirArriere 0x07
#define actRangerArriere 0x08
#define actResetCoucou 0x09
#define actFermerMoitieAvant 0x0A

#define actNoel 0x10
#define actIntimidation 0x20

#endif
