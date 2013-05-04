//#include <aversive.h>
#include <wait.h> //!
//#include <uart.h>
//#include <scheduler.h>
#include <i2cm.h> //!
//#include <adc.h>
//#include <math.h>
//#include "unioc_config.h"
#include "com.h" //!
//#include "position_manager.h"
//#include "asserv_manager.h"
//#include "trajectory_manager.h"


void arms(uint8_t ordre)
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
