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

#include <pid.h>


/** this function will initialize all fieds of pid structure to 0 */
void pid_init (struct pid_filter * p)
{
        p->gain_P = 1 ;
        p->gain_I = 0 ;
        p->gain_D = 0 ; 

        p->out_shift = 0 ;

        /* 0 means desactivated */
        p->max_in = 0 ;
        p->max_I = 0 ;
        p->max_out = 0 ;

        p->integral = 0 ;
        p->last_in = 0 ;
}

void pid_set_gains(struct pid_filter *p, int16_t gp, int16_t gi, int16_t gd)
{
    p->gain_P=gp;
    p->gain_I=gi;
    p->gain_D=gd;
}

void pid_set_maximums(struct pid_filter *p, int32_t max_in, int32_t max_I, int32_t max_out)
{
    p->max_in=max_in;
    p->max_I=max_I;
    p->max_out=max_out;
}

void pid_set_out_shift(struct pid_filter *p, int16_t out_shift)
{
    p->out_shift=out_shift;
}

int16_t pid_get_gain_P(struct pid_filter *p)
{
    return (p->gain_P);
}

int16_t pid_get_gain_I(struct pid_filter *p)
{
    return (p->gain_I);
}

int16_t pid_get_gain_D(struct pid_filter *p)
{
    return (p->gain_D);
}


int16_t pid_get_max_in(struct pid_filter *p)
{
    return (p->max_in);
}

int16_t pid_get_max_I(struct pid_filter *p)
{
    return (p->max_I);
}

int16_t pid_get_max_out(struct pid_filter *p)
{
    return (p->max_out);
}

int16_t pid_get_out_shift(struct pid_filter *p)
{
    return (p->out_shift);
}

int32_t pid_get_integral(struct pid_filter *p)
{
    return (p->integral);
}

int32_t pid_get_last_in(struct pid_filter *p)
{
    return (p->last_in);
}

/* first parameter should be a (struct pid_filter *) */
int32_t pid_do_filter(void * data, int32_t in)
{
   int32_t derivate ;
   int32_t command ;
   struct pid_filter * p = data;
   
   /* Integral value */
   /* The integral become bigger with time .. (think to area of graph : we add one area to the previous) */
   /* so, integral = previous integral + current value */
   p->integral += in ;

   if (p->max_I)
       S_MAX(p->integral, p->max_I) ;

   /* derivate value                                                */
   /*             f(t+h) - f(t)        with f(t+h) = current value  */
   /*  derivate = -------------             f(t)   = previous value */
   /*                    h                                          */
   /* so derivate = current error - previous error                  */
   derivate = in - p->last_in ;

   if (p->max_in)
       S_MAX(derivate, p->max_in) ;

   /* so, command = P+coef_P
                    + I*coef_I
                    + D*coef_D */
   command =    in * p->gain_P 
              + p->integral * p->gain_I
              + derivate * p->gain_D ;

   if ( command < 0 )
	   command = -( -command >> p->out_shift );
   else
	   command = command >> p->out_shift ;

   if (p->max_out)
       S_MAX (command, p->max_out) ;


   /* backup of current error value (for the next calcul of derivate value) */
   p->last_in = in ;
   
   return command;
} 
