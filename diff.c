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

#include <diff.h>

/** Prototyping */

void diff_init (struct diff * p)
{

  p->last_in = 0;

  p->first_call = 1;

  p->delta = 1;
  
  return;
};

/* Use these functions to change one parameter on pid_filter structure */
void diff_set_delta(struct diff *p,int32_t delta)
{
  p->delta = delta;

  return;
}

/** differential process */
int32_t diff_do_filter(void *p, int32_t in)
{

  struct diff* diff = p;
  
  if( diff->first_call )
  {

    diff->first_call = 0;

    diff->last_in = in;
    
    return 0;

  }
  else
  {

    int32_t tmp;
    
    tmp = ((in - diff->last_in))/(diff->delta);

    diff->last_in = in;

    diff->out = tmp;
    
    return tmp;

  }
}
        
        
