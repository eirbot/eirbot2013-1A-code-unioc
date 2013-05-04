#include <adc.h>


//EIRBUG

#define VOLT2ADC (0.1) // coefficient de convertion en 1/V

struct gp2_cfg_t {
  // Distance
  uint16_t dist_min;
  uint16_t dist_mid;
  uint16_t dist_max;

  // VOLT Input
  // ATTENTION : Unité en millivolt !
  uint16_t volt_min;
  uint16_t volt_mid;
  uint16_t volt_max;
};

// Configure l'approximation de la courbe du GP2
void gp2_init(struct gp2_cfg_t* gp2, uint16_t dmin, uint16_t dmid, uint16_t dmax, 
	 uint16_t voltmin, uint16_t voltmid, uint16_t voltmax);

// Donne la distance en cm de l'objet capté enfonction de la valeur de l'adc
uint16_t gp2_get_dist(struct gp2_cfg_t* gp2, uint16_t adc);

//////////////////////////////////////////////
//Ancien code
//uint8_t get_gp2( const uint32_t adc );
