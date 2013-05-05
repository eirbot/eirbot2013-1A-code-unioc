// fichier rajouté par Danaé pour la communication I2C avec la carte meca

#include <wait.h>
#include <i2cm.h>
#include "com.h"


void pince(uint8_t ordre)
{
	i2cm_send(2,1, &ordre);
	wait_ms(100);
	ordre = mecaBusy;
	while(ordre != mecaReady)
	{
			i2cm_rcv(2,1, &ordre);
			wait_ms(100);
	}


}

