#include <adc.h>


//EIRBUG

#define VOLT2ADC (0.2) // coefficient de convertion en 1/V

#define GP2_SAMPLES (10)

struct dist_adc {
  uint16_t dist;
  uint16_t adc;
};

struct gp2_cfg_t {
  uint8_t size;
  struct dist_adc points[GP2_SAMPLES];
};

// Configure l'approximation de la courbe du GP2
void gp2_init(struct gp2_cfg_t* gp2);

// Ajoute des poins à la courbe du gp2
void gp2_add_point(struct gp2_cfg_t* gp2, uint16_t dist, uint16_t volt);

// Donne la distance en cm de l'objet capté enfonction de la valeur de l'adc
uint16_t gp2_get_dist(struct gp2_cfg_t* gp2, uint16_t adc);

//////////////////////////////////////////////
//Ancien code
//uint8_t get_gp2( const uint32_t adc );
