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
struct diff
{
  int32_t last_in;

  uint8_t first_call;

  int32_t delta;

  int32_t out;
  
};

/** Prototyping */

void diff_init (struct diff * p);

/* Use these functions to change one parameter on pid_filter structure */
void diff_set_delta(struct diff *p,int32_t delta);

/** PID process */
int32_t diff_do_filter(void *p, int32_t in);
        
        
