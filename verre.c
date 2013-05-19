#include "verre.h"

#define GLASS_POS(i, _x, _y) \
  gm->glass[i].x= _x;	   \
  gm->glass[i].y= _y


void glassManagerInit(glass_manager* gm)
{
  gm->nb_glass_robot=0;
  gm->nb_glass_table=12;

  GLASS_POS(0, -60, 95);
  GLASS_POS(1, -30, 95);
  GLASS_POS(2, 30, 95);
  GLASS_POS(3, 60, 95);
  GLASS_POS(4, -45, 120);
  GLASS_POS(5, -15, 120);
  GLASS_POS(6, 15, 120);
  GLASS_POS(7, 45, 120);
  GLASS_POS(8, -60, 145);
  GLASS_POS(9, -30, 145);
  GLASS_POS(10, 30, 145);
  GLASS_POS(11, 60, 145);
}

glass_t milieu = {0,100};

glass_t* findClosest(glass_manager* gm, int16_t x, int16_t y)
{
  if(gm->nb_glass_table == 0) {
    return &milieu;
  }

  int8_t i,closest=0;
  uint16_t distance_temp,distance=-1;
  for(i=0 ; i < gm->nb_glass_table ; i++)
    {
      uint16_t dx = (x - gm->glass[i].x)/16;
      uint16_t dy = (y - gm->glass[i].y)/16;

      distance_temp= dx*dx+dy*dy;
      printf("\n x:%i , y=%i \n", gm->glass[i].x, gm->glass[i].y);
      printf("distance verre de %d, point %i\n", distance_temp, i);
      if(distance_temp < distance)
	{
	  distance = distance_temp;
	  closest = i;	  
	}
      
    }
  return &gm->glass[closest];
}


void removeGlass(glass_manager* gm, glass_t* gt)
{
  int32_t tmp_x,tmp_y;
  tmp_x=gm->glass[gm->nb_glass_table - 1].x;
  tmp_y=gm->glass[gm->nb_glass_table - 1].y;
  gm->glass[gm->nb_glass_table - 1].x=gt->x;
  gm->glass[gm->nb_glass_table - 1].y=gt->y;
  gt->x=tmp_x;
  gt->y=tmp_y;
  gm->nb_glass_table--;
}


uint8_t takeGlass(glass_manager* gm, glass_t* gt)
{
  removeGlass(gm,gt);
  gm->nb_glass_robot++;
  return gm->nb_glass_robot;
}
