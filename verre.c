#include "verre.h"

void glassManagerInit(glass_manager* gm)
{
  gm->nb_glass_robot=0;
  gm->nb_glass_table=0;
#define GLASS_POS(i, _x, _y) \
  gm->glass[i].x= _x;	   \
  gm->glass[i].y= _y

  GLASS_POS(0, -600, 950);
  GLASS_POS(1, -300, 950);
  GLASS_POS(2, 300, 950);
  GLASS_POS(3, 600, 950);
  GLASS_POS(4, -450, 1200);
  GLASS_POS(5, -150, 1200);
  GLASS_POS(6, 150, 1200);
  GLASS_POS(7, 450, 1200);
  GLASS_POS(8, -600, 1450);
  GLASS_POS(9, -300, 1450);
  GLASS_POS(10, 300, 1450);
  GLASS_POS(11, 600, 1450);
}

glass_t* findClosest(glass_manager* gm, uint16_t x, uint16_t y)
{
  int i,distance_temp;
  int closest=0;
  int distance=(x - gm->glass[0].x)*(x - gm->glass[0].x)+(y - gm->glass[0].y)*(y - gm->glass[0].y);
  for(i=1 ; i < gm->nb_glass_table ; i++)
    {
      distance_temp= (x - gm->glass[i].x)*(x - gm->glass[i].x)+(y - gm->glass[i].y)*(y - gm->glass[i].y);
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
  int tmp_x,tmp_y;
  tmp_x=gm->glass[gm->nb_glass_table - 1].x;
  tmp_y=gm->glass[gm->nb_glass_table - 1].y;
  gm->glass[gm->nb_glass_table - 1].x=gt->x;
  gm->glass[gm->nb_glass_table - 1].x=gt->y;
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
