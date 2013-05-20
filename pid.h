/*  
 *  Copyright Droids Corporation, Microb Technology, Eirbot (2006)
 * 
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *  Revision : $Id$
 *
 */

#include <utils.h>
#include <stdlib.h>

/** this is the pid_filter structure*/
struct pid_filter
{
    int16_t gain_P; /**< Gain of Proportionnal module */
    int16_t gain_I; /**< Gain of Integral module */
    int16_t gain_D; /**< Gain of Derivate module */

    int16_t out_shift;

    int32_t max_in; /**<  In saturation levels */
    int32_t max_I; /**<   Integral saturation levels */
    int32_t max_out; /**< Out saturation levels */

    int32_t integral; /**< previous integral parameter */
    int32_t last_in;  /**< previous derivate parameter */
};

/** Prototyping */

void pid_init (struct pid_filter * p);

/* Use these functions to change one parameter on pid_filter structure */
void pid_set_gains(struct pid_filter *p,int16_t gp, int16_t gi,int16_t gd) ;
void pid_set_maximums(struct pid_filter *p,int32_t max_in,int32_t max_I,int32_t max_out);
void pid_set_out_shift(struct pid_filter *p,int16_t out_shift);

/* accessors of all parameter of pid structure*/
int16_t pid_get_gain_P(struct pid_filter *p);
int16_t pid_get_gain_I(struct pid_filter *p);
int16_t pid_get_gain_D(struct pid_filter *p);
int16_t pid_get_max_in(struct pid_filter *p);
int16_t pid_get_max_I(struct pid_filter *p);
int16_t pid_get_max_out(struct pid_filter *p);
int16_t pid_get_out_shift(struct pid_filter *p);
int32_t pid_get_integral(struct pid_filter *p);
int32_t pid_get_last_in(struct pid_filter *p);

/** PID process */
int32_t pid_do_filter(void *p, int32_t in);
        
        
