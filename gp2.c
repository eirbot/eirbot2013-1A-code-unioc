#include "gp2.h"

void gp2_init(struct gp2_cfg_t* gp2, uint16_t dmin, uint16_t dmid, uint16_t dmax, 
	      uint16_t voltmin, uint16_t voltmid, uint16_t voltmax) {
  // Distance
  gp2->dist_min = dmin;
  gp2->dist_mid = dmid;
  gp2->dist_max = dmax;

  // VOLT Input
  gp2->volt_min = voltmin * VOLT2ADC;
  gp2->volt_mid = voltmid * VOLT2ADC;
  gp2->volt_max = voltmax * VOLT2ADC;
}

uint16_t gp2_get_dist(struct gp2_cfg_t* gp2, uint16_t adc) {
  adc = adc;
  if(adc < gp2->volt_mid) { // Grande distance
    // (dist - dist_mid) / (volt - volt_mid) = (dist_max - dist_mid) / (volt_max - volt_mid)
    return gp2->dist_mid + (gp2->dist_max - gp2->dist_mid) * (adc - gp2->volt_mid) / (gp2->volt_max - gp2->volt_mid);
  }
  else { // Petite distance
    // (dist - dist_mid) / (volt - volt_mid) = (dist_min - dist_mid) / (volt_min - volt_mid)
    return gp2->dist_mid + (gp2->dist_min - gp2->dist_mid) * (adc - gp2->volt_mid) / (gp2->volt_min - gp2->volt_mid);
  }
}

/*
uint8_t get_gp2( const uint32_t adc ) {
	if ( adc <= 650 && adc > 600 ) {
		return 7;
	}
	else if ( adc <= 600 && adc > 547 ) {
		return 8;
	}
	else if ( adc <= 547 && adc > 490 ) {
		return 9;
	}
	else if ( adc <= 490 && adc > 460 ) {
		return 10;
	}
	else if ( adc <= 460 && adc > 415 ) {
		return 11;
	}
	else if ( adc <= 415 && adc > 364 ) {
		return 12;
	}
	else if ( adc <= 364 && adc > 324 ) {
		return 14;
	}
	else if ( adc <= 324 && adc > 295 ) {
		return 16;
	}
	else if ( adc <= 295 && adc > 270 ) {
		return 18;
	}
	else if ( adc <= 270 && adc > 245 ) {
		return 20;
	}
	else if ( adc <= 245 && adc > 222 ) {
		return 22;
	}
	else if ( adc <= 222 && adc > 197 ) {
		return 24;
	}
	else if ( adc <= 197 && adc > 190 ) {
		return 26;
	}
	else if ( adc <= 190 && adc > 170 ) {
		return 28;
	}
	else if ( adc <= 170 && adc > 166 ) {
		return 30;
	}
	else if ( adc <= 166 && adc > 158 ) {
		return 32;
	}
	else if ( adc <= 158 && adc > 150 ) {
		return 34;
	}
	else if ( adc <= 150 && adc > 137 ) {
		return 36;
	}
	else if ( adc <= 137 && adc > 124 ) {
		return 38;
	}
	else if ( adc <= 124 && adc > 120 ) {
		return 40;
	}
	else if ( adc <= 120 && adc > 108 ) {
		return 42;
	}
	else if ( adc <= 108 && adc > 102 ) {
		return 44;
	}
	else if ( adc <= 102 && adc > 78 ) {
		return 46;
	}
	else if ( adc <= 78 && adc > 65 ) {
		return 48;
	}
	else if ( adc <= 65 && adc > 66 ) {
		return 50;
	}
	else if ( adc <= 66 && adc > 39 ) {
		return 52;
	}
	else if ( adc <= 39 && adc > 10 ) {
		return 60;
	}
	else{
		return 255;
	}
}
*/


/*
uint8_t get_gp2( const uint32_t adc ) {
	uint64_t tmp = 0;
	int64_t result = 0;
	uint8_t retour = 0;

	if ( adc < 30 || adc > 650 ) {
	       return 255;
	}	       

	tmp = 343*adc*adc*adc;
	tmp /= 1000000000;

	result -= tmp;
	result += 70;

	tmp = 55*adc*adc;
	tmp /= 100000;

	result += tmp;

	tmp = 31*adc;
	tmp /= 100;

	result -= tmp; 

	retour = (uint8_t)result;

	return retour;
}
*/	

































