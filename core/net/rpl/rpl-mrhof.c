/**
 * \addtogroup uip6
 * @{
 */
/*
 * Copyright (c) 2010, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */
/**
 * \file
 *         The Minimum Rank with Hysteresis Objective Function (MRHOF)
 *
 *         This implementation uses the estimated number of 
 *         transmissions (ETX) as the additive routing metric,
 *         and also provides stubs for the energy metric.
 *
 * \author Joakim Eriksson <joakime@sics.se>, Nicolas Tsiftes <nvt@sics.se>
 */

#include "net/rpl/rpl-private.h"
#include "net/nbr-table.h"
#include "dev/cc2420.h"//elnaz
#define DEBUG DEBUG_NONE
#include "net/uip-debug.h"
//elnaz
static unsigned long last_probe_time=0;
static char etx_flag=0;
static char start_flag=1;
//static char steps=0;
static char wait=10;
//elnaz

static void reset(rpl_dag_t *);
static void neighbor_link_callback(rpl_parent_t *, int, int);
static rpl_parent_t *best_parent(rpl_parent_t *, rpl_parent_t *);
static rpl_dag_t *best_dag(rpl_dag_t *, rpl_dag_t *);
static rpl_rank_t calculate_rank(rpl_parent_t *, rpl_rank_t);
static void update_metric_container(rpl_instance_t *);

rpl_of_t rpl_mrhof = {
  reset,
  neighbor_link_callback,
  best_parent,
  best_dag,
  calculate_rank,
  update_metric_container,
  1
};

/* Constants for the ETX moving average */
#define ETX_SCALE   100
#define ETX_ALPHA   90


/* Reject parents that have a higher link metric than the following. */
#define MAX_LINK_METRIC			10

/* Reject parents that have a higher path cost than the following. */
#define MAX_PATH_COST			100

/*
 * The rank must differ more than 1/PARENT_SWITCH_THRESHOLD_DIV in order
 * to switch preferred parent.
 */
#define PARENT_SWITCH_THRESHOLD_DIV	2

typedef uint16_t rpl_path_metric_t;

static rpl_path_metric_t
calculate_path_metric(rpl_parent_t *p)
{
  if(p == NULL) {
    return MAX_PATH_COST * RPL_DAG_MC_ETX_DIVISOR;
  }

#if RPL_DAG_MC == RPL_DAG_MC_NONE
  return p->rank + (uint16_t)p->link_metric;
#elif RPL_DAG_MC == RPL_DAG_MC_ETX
  return p->mc.obj.etx + (uint16_t)p->link_metric;
#elif RPL_DAG_MC == RPL_DAG_MC_ENERGY
  return p->mc.obj.energy.energy_est + (uint16_t)p->link_metric;
#else
#error "Unsupported RPL_DAG_MC configured. See rpl.h."
#endif /* RPL_DAG_MC */
}

//elnaz
static uint16_t ETX_THERESHOLD=(RPL_DAG_MC_ETX_DIVISOR*1.5);
//elnaz
static void
reset(rpl_dag_t *sag)
{
  PRINTF("RPL: Reset MRHOF\n");
}
/*----------------------------------------------------------*/
void adjust_ETX_th(void)
{


etx_flag=1;

}
/*-----------------------------------------------------------*/
void set_ETX_th(void)
{
	int tx= cc2420_get_txpower();
	if(tx==3 || tx==7)
	{ETX_THERESHOLD=8*RPL_DAG_MC_ETX_DIVISOR;}
	else if(tx>10)// && tx<20)
	{ETX_THERESHOLD = 2 *RPL_DAG_MC_ETX_DIVISOR;}
	//else if(tx>20 && tx<32)
	//{ETX_THERESHOLD = 1.5;}
//printf("set ETX_th=%d\n",ETX_THERESHOLD);
	

}
/*-----------------------------------------------------------*/
static void
neighbor_link_callback(rpl_parent_t *p, int status, int numtx)
{
  uint16_t recorded_etx = p->link_metric;
  uint16_t packet_etx = numtx * RPL_DAG_MC_ETX_DIVISOR;
  uint16_t new_etx;
  uint16_t temp;
uip_ipaddr_t *dest;
dest=rpl_get_parent_ipaddr(p);

//elnaz
unsigned long delta;
rpl_parent_t * pref;
p->numtx = p->numtx + numtx;
p->recv++;
//elnaz
  /* Do not penalize the ETX when collisions or transmission errors occur. */
  if(status == MAC_TX_OK || status == MAC_TX_NOACK) {
    if(status == MAC_TX_NOACK) {
      packet_etx = MAX_LINK_METRIC * RPL_DAG_MC_ETX_DIVISOR;

//printf("no ack %02x%02x \n",((uint8_t *)dest)[14], ((uint8_t *)dest)[15]);
    }
dest=rpl_get_parent_ipaddr(p);
//printf("numtx=%d ADDR=%02x%02x \n", numtx, ((uint8_t *)dest)[14], ((uint8_t *)dest)[15]);

    new_etx = ((uint32_t)recorded_etx * ETX_ALPHA +
               (uint32_t)packet_etx * (ETX_SCALE - ETX_ALPHA)) / ETX_SCALE;

    PRINTF("RPL: ETX changed from %u to %u (packet ETX = %u)\n",
        (unsigned)(recorded_etx / RPL_DAG_MC_ETX_DIVISOR),
        (unsigned)(new_etx  / RPL_DAG_MC_ETX_DIVISOR),
        (unsigned)(packet_etx / RPL_DAG_MC_ETX_DIVISOR));
    p->link_metric = new_etx;

//elnaz
//----------------------- PROBE 

   if(start_flag==1)
	{last_probe_time=clock_seconds();
	start_flag=0;
	set_ETX_th();}

//test = PROBE_NUM_THRESHOLD * PROBE_INTERVAL * 2;
//printf("%lu , test= %d, delta=%d ", (clock_seconds()-last_probe_time),  test, (PROBE_NUM_THRESHOLD * PROBE_INTERVAL * 2) );
  pref = p->dag->preferred_parent;
  temp = (p->link_metric)/RPL_DAG_MC_ETX_DIVISOR;
  temp = temp * RPL_DAG_MC_ETX_DIVISOR;
  if(temp>ETX_THERESHOLD && p==pref)
	{	printf("pass threshhold ADDR=%02x etx=%u th=%u\n", ((uint8_t *)dest)[15],p->link_metric,ETX_THERESHOLD);
		delta = (clock_seconds()-last_probe_time);
		//printf("t=%lu , %lu,%lu, %lu \n",delta, delta*CLOCK_SECOND, 120, 120*CLOCK_SECOND);
	if(delta>(wait*60))
		{	//printf("pass time constraint");
			wait=5;
			if(etx_flag==1)
			{
				ETX_THERESHOLD = pref->link_metric;
				etx_flag=0;
//				printf("adjust etx-th \n");
				
			}
			else if(find_pref()==1)
			{	last_probe_time=clock_seconds();
//				printf("update time\n");
				
				//steps=0;	
			}

		}
//	if(delta>=)

	else
		{	/*steps++;
			if(steps==11)
			{ adjust_ETX_th(p->link_metric);
			  steps=0;
			 }*/
			//steps=0; 
			//printf("not if");	
		}
	}
   
  }
else
	{ //printf("error ADDR= %02x%02x \n",((uint8_t *)dest)[14], ((uint8_t *)dest)[15]);
	}
monitor_parents();
}

/*----------------------------------------------------------------------------*/

static rpl_rank_t
calculate_rank(rpl_parent_t *p, rpl_rank_t base_rank)
{
  rpl_rank_t new_rank;
  rpl_rank_t rank_increase;

  if(p == NULL) {
    if(base_rank == 0) {
      return INFINITE_RANK;
    }
    rank_increase = RPL_INIT_LINK_METRIC * RPL_DAG_MC_ETX_DIVISOR;
  } else {
    rank_increase = p->link_metric;
    if(base_rank == 0) {
      base_rank = p->rank;
    }
  }

  if(INFINITE_RANK - base_rank < rank_increase) {
    /* Reached the maximum rank. */
    new_rank = INFINITE_RANK;
  } else {
   /* Calculate the rank based on the new rank information from DIO or
      stored otherwise. */
    new_rank = base_rank + rank_increase;
  }

  return new_rank;
}

static rpl_dag_t *
best_dag(rpl_dag_t *d1, rpl_dag_t *d2)
{
  if(d1->grounded != d2->grounded) {
    return d1->grounded ? d1 : d2;
  }

  if(d1->preference != d2->preference) {
    return d1->preference > d2->preference ? d1 : d2;
  }

  return d1->rank < d2->rank ? d1 : d2;
}

static rpl_parent_t *
best_parent(rpl_parent_t *p1, rpl_parent_t *p2)
{
  rpl_dag_t *dag;
  rpl_path_metric_t min_diff;
  rpl_path_metric_t p1_metric;
  rpl_path_metric_t p2_metric;

  dag = p1->dag; /* Both parents are in the same DAG. */

  min_diff = RPL_DAG_MC_ETX_DIVISOR /
             PARENT_SWITCH_THRESHOLD_DIV;

  p1_metric = calculate_path_metric(p1);
  p2_metric = calculate_path_metric(p2);

  /* Maintain stability of the preferred parent in case of similar ranks. */
  if(p1 == dag->preferred_parent || p2 == dag->preferred_parent) {
    if(p1_metric < p2_metric + min_diff &&
       p1_metric > p2_metric - min_diff) {
      PRINTF("RPL: MRHOF hysteresis: %u <= %u <= %u\n",
             p2_metric - min_diff,
             p1_metric,
             p2_metric + min_diff);
      return dag->preferred_parent;
    }
  }

  return p1_metric < p2_metric ? p1 : p2;
}

#if RPL_DAG_MC == RPL_DAG_MC_NONE
static void
update_metric_container(rpl_instance_t *instance)
{
  instance->mc.type = RPL_DAG_MC;
}
#else
static void
update_metric_container(rpl_instance_t *instance)
{
  rpl_path_metric_t path_metric;
  rpl_dag_t *dag;
#if RPL_DAG_MC == RPL_DAG_MC_ENERGY
  uint8_t type;
#endif

  instance->mc.type = RPL_DAG_MC;
  instance->mc.flags = RPL_DAG_MC_FLAG_P;
  instance->mc.aggr = RPL_DAG_MC_AGGR_ADDITIVE;
  instance->mc.prec = 0;

  dag = instance->current_dag;

  if (!dag->joined) {
    PRINTF("RPL: Cannot update the metric container when not joined\n");
    return;
  }

  if(dag->rank == ROOT_RANK(instance)) {
    path_metric = 0;
  } else {
    path_metric = calculate_path_metric(dag->preferred_parent);
  }

#if RPL_DAG_MC == RPL_DAG_MC_ETX
  instance->mc.length = sizeof(instance->mc.obj.etx);
  instance->mc.obj.etx = path_metric;

  PRINTF("RPL: My path ETX to the root is %u.%u\n",
	instance->mc.obj.etx / RPL_DAG_MC_ETX_DIVISOR,
	(instance->mc.obj.etx % RPL_DAG_MC_ETX_DIVISOR * 100) /
	 RPL_DAG_MC_ETX_DIVISOR);
#elif RPL_DAG_MC == RPL_DAG_MC_ENERGY
  instance->mc.length = sizeof(instance->mc.obj.energy);

  if(dag->rank == ROOT_RANK(instance)) {
    type = RPL_DAG_MC_ENERGY_TYPE_MAINS;
  } else {
    type = RPL_DAG_MC_ENERGY_TYPE_BATTERY;
  }

  instance->mc.obj.energy.flags = type << RPL_DAG_MC_ENERGY_TYPE;
  instance->mc.obj.energy.energy_est = path_metric;
#endif /* RPL_DAG_MC == RPL_DAG_MC_ETX */
}
#endif /* RPL_DAG_MC == RPL_DAG_MC_NONE */
