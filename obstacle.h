#ifndef _OBSTACLE_H_
#define _OBSTACLE_H_

enum obstacle_t {RIEN = 0, VERRE, GATEAU, WALL, ROBOT};

/*
   +-> x
   |
   v y
          (0,0)
|------\  +  /------|
|H      \___/      H|
|H                 H|
|H                 H|
|H                 H|
|___________________|

H : assiete
 */

enum figure {RECT, CIRCLE};

struct obstacle {
  enum figure fig;
  int16_t x;
  int16_t y;
  int16_t dx;
  int16_t dy;
  int16_t dr;
};

// Configuration
#define TABLE_WIDTH 300
#define TABLE_HEIGHT 200
#define GATEAU_RAY 70

#define ASSIETE_W 17
#define ASSIETE_DIST_X (20 - ASSIETE_W/2)
#define ASSIETE_DIST_Y (25 - ASSIETE_W/2)
#define ASSIETE_DIST_INTER (60 - 25 - ASSIETE_W)

#define INFINITE 1000

// Macros Utiles
#define MK_RECT_WH(x, y, w, h) {RECT, x, y, w, h, 0}
#define MK_RECT_2P(x1, y1, x2, y2) {RECT, x1, y1, x2 - x1, y2 - y1, 0}
#define MK_CIRCLE(x, y, r) {RECT, x, y, 0, 0, r}


#define MK_ASSIETE_O(i) \ 
MK_RECT_WH(-TABLE_WIDTH/2 + ASSIETE_DIST_X , ASSIETE_DIST_Y + i * (ASSIETE_DIST_INTER + ASSIETE_W), ASSIETE_W, ASSIETE_W, 0)
#define MK_ASSIETE_E(i) \
  MK_RECT_WH(TABLE_WIDTH/2 - (ASSIETE_DIST_X + ASSIETE_W) , ASSIETE_DIST_Y + i * (ASSIETE_DIST_INTER + ASSIETE_W), ASSIETE_W, ASSIETE_W, 0)

// Obstacles
struct obstacle obstacles[] = {
  //// Murs de la table
  MK_RECT_2P(-INFINITE, -INFINITE, INFINITE, 0) // Nord
  {RECT, -INFINITE,                 -INFINITE,    2*INFINITE, INFINITE,   0}, // Nord
  {RECT, -INFINITE - TABLE_WIDTH/2, -INFINITE,    INFINITE,   2*INFINITE, 0}, // Ouest
  {RECT, -INFINITE,                 TABLE_HEIGHT, 2*INFINITE, INFINITE,   0}, // Sud
  {RECT, TABLE_WIDTH/2,             -INFINITE,    INFINITE,   2*INFINITE, 0}, // Est
  //// Gateau
  MK_CIRCLE(0, 0, GATEAU_RAY)
  //// Assietes
  //OSEF !
};

//retourne 1 si l'obstacle vu est un obstacle connu du plateau 0 sinon, retourne par effet de bord le type de l'objet
obstacle_t knownObstacle(int16_t x, int16_t y, int16_t a);




#endif //_OBSTACLE_H_
