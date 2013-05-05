#include "gp2.h"

void gp2_init(struct gp2_cfg_t* gp2) {
  gp2->size = 0;
}


void gp2_add_point(struct gp2_cfg_t* gp2, uint16_t dist, uint16_t volt) {
  struct dist_adc e = {dist, volt * VOLT2ADC};
  struct dist_adc tmp;

  if(gp2->size >= GP2_SAMPLES)
    return;

  gp2->size++;
  for(int i = 0 ; i < gp2->size ; i++) {
    if(gp2->points[i].adc >= e.adc) {
	tmp = gp2->points[i];
	gp2->points[i] = e;
	e = tmp;
    }
  }
}


uint16_t gp2_get_dist(struct gp2_cfg_t* gp2, uint16_t adc) {
  for(int i = 0 ; i < gp2->size ; i++) {
    if(gp2->points[i].adc >= adc) {
      return gp2->points[i].dist;
    }
  }
  return 0;
}
