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
 *         Logic for Directed Acyclic Graphs in RPL.
 *
 * \author Joakim Eriksson <joakime@sics.se>, Nicolas Tsiftes <nvt@sics.se>
 */

#include "dev/cc2420.h"//elnaz
#include "lib/random.h"
#include "contiki.h"
#include "net/rpl/rpl-private.h"
#include "net/uip.h"
#include "net/uip-nd6.h"
#include "net/nbr-table.h"
#include "lib/list.h"
#include "lib/memb.h"
#include "sys/ctimer.h"
//#include "math.h"
#include <limits.h>
#include <string.h>

#define DEBUG DEBUG_PRINT
#include "net/uip-debug.h"

#if UIP_CONF_IPV6
/*---------------------------------------------------------------------------*/
extern rpl_of_t RPL_OF;
static rpl_of_t * const objective_functions[] = {&RPL_OF};

/*---------------------------------------------------------------------------*/
/* RPL definitions. */

#ifndef RPL_CONF_GROUNDED
#define RPL_GROUNDED                    0
#else
#define RPL_GROUNDED                    RPL_CONF_GROUNDED
#endif /* !RPL_CONF_GROUNDED */

#define RSSI_THRESHOLD -90
#define hexa 64
#define AGE_THRESHOLD 1049//(2^(RPL_DIO_INTERVAL_MIN ))/hexa //+ RPL_DIO_INTERVAL_DOUBLINGS
//elnaz
static struct ip_addr_list_struct pt_parents[3];
static void handle_probe_timer(void *pt);
//LIST(ip_addr_list);
//MEMB(pt_prnt_mem, struct ip_addr_list_struct, 3);

static int probe_number=0;
static int probe_total=0;
static struct ctimer probe_timer;
static struct ctimer  stagger_timer;
static struct ctimer  find_pref_timer;
static void handle_probe_timer(void *);
static void handle_stagger_timer(void *);
static int index;

static int Tx_array[8]={3,7,11,15,19,23,27,31};
static int Es_array[8]={330,330,330,330,330,330,330,330};
//static char flag=0;
//elnaz
/*---------------------------------------------------------------------------*/
/* Per-parent RPL information */
NBR_TABLE(rpl_parent_t, rpl_parents);
/*---------------------------------------------------------------------------*/
/* Allocate instance table. */
rpl_instance_t instance_table[RPL_MAX_INSTANCES];
rpl_instance_t *default_instance;
/*---------------------------------------------------------------------------*/
void
rpl_dag_init(void)
{
  nbr_table_register(rpl_parents, (nbr_table_callback *)rpl_remove_parent);
}
/*---------------------------------------------------------------------------*/
rpl_rank_t
rpl_get_parent_rank(uip_lladdr_t *addr)
{
  rpl_parent_t *p = nbr_table_get_from_lladdr(rpl_parents, (rimeaddr_t *)addr);
  if(p != NULL) {
    return p->rank;
  } else {
    return 0;
  }
}
/*---------------------------------------------------------------------------*/
uint16_t
rpl_get_parent_link_metric(uip_lladdr_t *addr)
{
  rpl_parent_t *p = nbr_table_get_from_lladdr(rpl_parents, (rimeaddr_t *)addr);
  if(p != NULL) {
    return p->link_metric;
  } else {
    return 0;
  }
}
/*---------------------------------------------------------------------------*/
uip_ipaddr_t *
rpl_get_parent_ipaddr(rpl_parent_t *p)
{
  rimeaddr_t *lladdr = nbr_table_get_lladdr(rpl_parents, p);
  return uip_ds6_nbr_ipaddr_from_lladdr((uip_lladdr_t *)lladdr);
}
/*---------------------------------------------------------------------------*/
static void
rpl_set_preferred_parent(rpl_dag_t *dag, rpl_parent_t *p)
{
  if(dag != NULL && dag->preferred_parent != p) {
   // PRINTF("RPL: rpl_set_preferred_parent ");
    if(p != NULL) {
      PRINT6ADDR(rpl_get_parent_ipaddr(p));
    } else {
    //  PRINTF("NULL");
    }
   // PRINTF(" used to be ");
    if(dag->preferred_parent != NULL) {
      PRINT6ADDR(rpl_get_parent_ipaddr(dag->preferred_parent));
    } else {
   //   PRINTF("NULL");
    }
  //  PRINTF("\n");

    /* Always keep the preferred parent locked, so it remains in the
     * neighbor table. */
    nbr_table_unlock(rpl_parents, dag->preferred_parent);
    nbr_table_lock(rpl_parents, p);
    dag->preferred_parent = p;
    p->flag = Pref_PRNT;
  }
}
/*---------------------------------------------------------------------------*/
/* Greater-than function for the lollipop counter.                      */
/*---------------------------------------------------------------------------*/
static int
lollipop_greater_than(int a, int b)
{
  /* Check if we are comparing an initial value with an old value */
  if(a > RPL_LOLLIPOP_CIRCULAR_REGION && b <= RPL_LOLLIPOP_CIRCULAR_REGION) {
    return (RPL_LOLLIPOP_MAX_VALUE + 1 + b - a) > RPL_LOLLIPOP_SEQUENCE_WINDOWS;
  }
  /* Otherwise check if a > b and comparable => ok, or
     if they have wrapped and are still comparable */
  return (a > b && (a - b) < RPL_LOLLIPOP_SEQUENCE_WINDOWS) ||
    (a < b && (b - a) > (RPL_LOLLIPOP_CIRCULAR_REGION + 1-
			 RPL_LOLLIPOP_SEQUENCE_WINDOWS));
}
/*---------------------------------------------------------------------------*/
/* Remove DAG parents with a rank that is at least the same as minimum_rank. */
static void
remove_parents(rpl_dag_t *dag, rpl_rank_t minimum_rank)
{
  rpl_parent_t *p;

 // PRINTF("RPL: Removing parents (minimum rank %u)\n", minimum_rank);

  p = nbr_table_head(rpl_parents);
  while(p != NULL) {
    if(dag == p->dag && p->rank >= minimum_rank) {
      rpl_remove_parent(p);
    }
    p = nbr_table_next(rpl_parents, p);
  }
}
/*---------------------------------------------------------------------------*/
static void
nullify_parents(rpl_dag_t *dag, rpl_rank_t minimum_rank)
{
  rpl_parent_t *p;

//  PRINTF("RPL: Removing parents (minimum rank %u)\n",minimum_rank);

  p = nbr_table_head(rpl_parents);
  while(p != NULL) {
    if(dag == p->dag && p->rank >= minimum_rank) {
      rpl_nullify_parent(p);
    }
    p = nbr_table_next(rpl_parents, p);
  }
}
/*---------------------------------------------------------------------------*/
static int
should_send_dao(rpl_instance_t *instance, rpl_dio_t *dio, rpl_parent_t *p)
{
  /* if MOP is set to no downward routes no DAO should be sent */
  if(instance->mop == RPL_MOP_NO_DOWNWARD_ROUTES) {
    return 0;
  }
  /* check if the new DTSN is more recent */
  return p == instance->current_dag->preferred_parent &&
    (lollipop_greater_than(dio->dtsn, p->dtsn));
}
/*---------------------------------------------------------------------------*/
static int
acceptable_rank(rpl_dag_t *dag, rpl_rank_t rank)
{
  return rank != INFINITE_RANK &&
    ((dag->instance->max_rankinc == 0) ||
     DAG_RANK(rank, dag->instance) <= DAG_RANK(dag->min_rank + dag->instance->max_rankinc, dag->instance));
}
/*---------------------------------------------------------------------------*/
static rpl_dag_t *
get_dag(uint8_t instance_id, uip_ipaddr_t *dag_id)
{
  rpl_instance_t *instance;
  rpl_dag_t *dag;
  int i;

  instance = rpl_get_instance(instance_id);
  if(instance == NULL) {
    return NULL;
  }

  for(i = 0; i < RPL_MAX_DAG_PER_INSTANCE; ++i) {
    dag = &instance->dag_table[i];
    if(dag->used && uip_ipaddr_cmp(&dag->dag_id, dag_id)) {
      return dag;
    }
  }

  return NULL;
}
/*---------------------------------------------------------------------------*/
rpl_dag_t *
rpl_set_root(uint8_t instance_id, uip_ipaddr_t *dag_id)
{
  rpl_dag_t *dag;
  rpl_instance_t *instance;
  uint8_t version;

  version = RPL_LOLLIPOP_INIT;
  dag = get_dag(instance_id, dag_id);
  if(dag != NULL) {
    version = dag->version;
    RPL_LOLLIPOP_INCREMENT(version);
//    PRINTF("RPL: Dropping a joined DAG when setting this node as root");
    if(dag == dag->instance->current_dag) {
      dag->instance->current_dag = NULL;
    }
    rpl_free_dag(dag);
  }

  dag = rpl_alloc_dag(instance_id, dag_id);
  if(dag == NULL) {
//    PRINTF("RPL: Failed to allocate a DAG\n");
    return NULL;
  }

  instance = dag->instance;

  dag->version = version;
  dag->joined = 1;
  dag->grounded = RPL_GROUNDED;
  instance->mop = RPL_MOP_DEFAULT;
  instance->of = &RPL_OF;
  rpl_set_preferred_parent(dag, NULL);

  memcpy(&dag->dag_id, dag_id, sizeof(dag->dag_id));

  instance->dio_intdoubl = RPL_DIO_INTERVAL_DOUBLINGS;
  instance->dio_intmin = RPL_DIO_INTERVAL_MIN;
  /* The current interval must differ from the minimum interval in order to
     trigger a DIO timer reset. */
  instance->dio_intcurrent = RPL_DIO_INTERVAL_MIN +
    RPL_DIO_INTERVAL_DOUBLINGS;
  instance->dio_redundancy = RPL_DIO_REDUNDANCY;
  instance->max_rankinc = RPL_MAX_RANKINC;
  instance->min_hoprankinc = RPL_MIN_HOPRANKINC;
  instance->default_lifetime = RPL_DEFAULT_LIFETIME;
  instance->lifetime_unit = RPL_DEFAULT_LIFETIME_UNIT;

  dag->rank = ROOT_RANK(instance);

  if(instance->current_dag != dag && instance->current_dag != NULL) {
    /* Remove routes installed by DAOs. */
    rpl_remove_routes(instance->current_dag);

    instance->current_dag->joined = 0;
  }

  instance->current_dag = dag;
  instance->dtsn_out = RPL_LOLLIPOP_INIT;
  instance->of->update_metric_container(instance);
  default_instance = instance;

//  PRINTF("RPL: Node set to be a DAG root with DAG ID ");
//  PRINT6ADDR(&dag->dag_id);
//  PRINTF("\n");

  ANNOTATE("#A root=%u\n", dag->dag_id.u8[sizeof(dag->dag_id) - 1]);

  rpl_reset_dio_timer(instance);

  return dag;
}
/*---------------------------------------------------------------------------*/
int
rpl_repair_root(uint8_t instance_id)
{
  rpl_instance_t *instance;

  instance = rpl_get_instance(instance_id);
  if(instance == NULL ||
     instance->current_dag->rank != ROOT_RANK(instance)) {
 //   PRINTF("RPL: rpl_repair_root triggered but not root\n");
    return 0;
  }

  RPL_LOLLIPOP_INCREMENT(instance->current_dag->version);
  RPL_LOLLIPOP_INCREMENT(instance->dtsn_out);
//  PRINTF("RPL: rpl_repair_root initiating global repair with version %d\n", instance->current_dag->version);
  rpl_reset_dio_timer(instance);
  return 1;
}
/*---------------------------------------------------------------------------*/
static void
set_ip_from_prefix(uip_ipaddr_t *ipaddr, rpl_prefix_t *prefix)
{
  memset(ipaddr, 0, sizeof(uip_ipaddr_t));
  memcpy(ipaddr, &prefix->prefix, (prefix->length + 7) / 8);
  uip_ds6_set_addr_iid(ipaddr, &uip_lladdr);
}
/*---------------------------------------------------------------------------*/
static void
check_prefix(rpl_prefix_t *last_prefix, rpl_prefix_t *new_prefix)
{
  uip_ipaddr_t ipaddr;
  uip_ds6_addr_t *rep;

  if(last_prefix != NULL && new_prefix != NULL &&
     last_prefix->length == new_prefix->length &&
     uip_ipaddr_prefixcmp(&last_prefix->prefix, &new_prefix->prefix, new_prefix->length) &&
     last_prefix->flags == new_prefix->flags) {
    /* Nothing has changed. */
    return;
  }

  if(last_prefix != NULL) {
    set_ip_from_prefix(&ipaddr, last_prefix);
    rep = uip_ds6_addr_lookup(&ipaddr);
    if(rep != NULL) {
//      PRINTF("RPL: removing global IP address ");
//      PRINT6ADDR(&ipaddr);
//      PRINTF("\n");
      uip_ds6_addr_rm(rep);
    }
  }
  
  if(new_prefix != NULL) {
    set_ip_from_prefix(&ipaddr, new_prefix);
    if(uip_ds6_addr_lookup(&ipaddr) == NULL) {
      PRINTF("RPL: adding global IP address ");
      PRINT6ADDR(&ipaddr);
      PRINTF("\n");
      uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);
    }
  }
}
/*---------------------------------------------------------------------------*/
int
rpl_set_prefix(rpl_dag_t *dag, uip_ipaddr_t *prefix, unsigned len)
{
  rpl_prefix_t last_prefix;
  uint8_t last_len = dag->prefix_info.length;
  
  if(len > 128) {
    return 0;
  }
  if(dag->prefix_info.length != 0) {
    memcpy(&last_prefix, &dag->prefix_info, sizeof(rpl_prefix_t));
  }
  memset(&dag->prefix_info.prefix, 0, sizeof(dag->prefix_info.prefix));
  memcpy(&dag->prefix_info.prefix, prefix, (len + 7) / 8);
  dag->prefix_info.length = len;
  dag->prefix_info.flags = UIP_ND6_RA_FLAG_AUTONOMOUS;
//  PRINTF("RPL: Prefix set - will announce this in DIOs\n");
  /* Autoconfigure an address if this node does not already have an address
     with this prefix. Otherwise, update the prefix */
  if(last_len == 0) {
//    PRINTF("rpl_set_prefix - prefix NULL\n");
    check_prefix(NULL, &dag->prefix_info);
  } else { 
//    PRINTF("rpl_set_prefix - prefix NON-NULL\n");
    check_prefix(&last_prefix, &dag->prefix_info);
  }
  return 1;
}
/*---------------------------------------------------------------------------*/
int
rpl_set_default_route(rpl_instance_t *instance, uip_ipaddr_t *from)
{
  if(instance->def_route != NULL) {
//    PRINTF("RPL: Removing default route through ");
//    PRINT6ADDR(&instance->def_route->ipaddr);
//    PRINTF("\n");
    uip_ds6_defrt_rm(instance->def_route);
    instance->def_route = NULL;
  }

  if(from != NULL) {
//    PRINTF("RPL: Adding default route through ");
//    PRINT6ADDR(from);
//    PRINTF("\n");
    instance->def_route = uip_ds6_defrt_add(from,
        RPL_LIFETIME(instance,
            instance->default_lifetime));
    if(instance->def_route == NULL) {
      return 0;
    }
  } else {
//    PRINTF("RPL: Removing default route\n");
    if(instance->def_route != NULL) {
      uip_ds6_defrt_rm(instance->def_route);
    } else {
//      PRINTF("RPL: Not actually removing default route, since instance had no default route\n");
    }
  }
  return 1;
}
/*---------------------------------------------------------------------------*/
rpl_instance_t *
rpl_alloc_instance(uint8_t instance_id)
{
  rpl_instance_t *instance, *end;

  for(instance = &instance_table[0], end = instance + RPL_MAX_INSTANCES;
      instance < end; ++instance) {
    if(instance->used == 0) {
      memset(instance, 0, sizeof(*instance));
      instance->instance_id = instance_id;
      instance->def_route = NULL;
      instance->used = 1;
      return instance;
    }
  }
  return NULL;
}
/*---------------------------------------------------------------------------*/
rpl_dag_t *
rpl_alloc_dag(uint8_t instance_id, uip_ipaddr_t *dag_id)
{
  rpl_dag_t *dag, *end;
  rpl_instance_t *instance;

  instance = rpl_get_instance(instance_id);
  if(instance == NULL) {
    instance = rpl_alloc_instance(instance_id);
    if(instance == NULL) {
      RPL_STAT(rpl_stats.mem_overflows++);
      return NULL;
    }
  }

  for(dag = &instance->dag_table[0], end = dag + RPL_MAX_DAG_PER_INSTANCE; dag < end; ++dag) {
    if(!dag->used) {
      memset(dag, 0, sizeof(*dag));
      dag->used = 1;
      dag->rank = INFINITE_RANK;
      dag->min_rank = INFINITE_RANK;
      dag->instance = instance;
      return dag;
    }
  }

  RPL_STAT(rpl_stats.mem_overflows++);
  rpl_free_instance(instance);
  return NULL;
}
/*---------------------------------------------------------------------------*/
void
rpl_set_default_instance(rpl_instance_t *instance)
{
  default_instance = instance;
}
/*---------------------------------------------------------------------------*/
void
rpl_free_instance(rpl_instance_t *instance)
{
  rpl_dag_t *dag;
  rpl_dag_t *end;

//  PRINTF("RPL: Leaving the instance %u\n", instance->instance_id);

  /* Remove any DAG inside this instance */
  for(dag = &instance->dag_table[0], end = dag + RPL_MAX_DAG_PER_INSTANCE; dag < end; ++dag) {
    if(dag->used) {
      rpl_free_dag(dag);
    }
  }

  rpl_set_default_route(instance, NULL);

  ctimer_stop(&instance->dio_timer);
  ctimer_stop(&instance->dao_timer);

  if(default_instance == instance) {
    default_instance = NULL;
  }

  instance->used = 0;
}
/*---------------------------------------------------------------------------*/
void
rpl_free_dag(rpl_dag_t *dag)
{
  if(dag->joined) {
//    PRINTF("RPL: Leaving the DAG ");
//    PRINT6ADDR(&dag->dag_id);
//    PRINTF("\n");
    dag->joined = 0;

    /* Remove routes installed by DAOs. */
    rpl_remove_routes(dag);

   /* Remove autoconfigured address */
    if((dag->prefix_info.flags & UIP_ND6_RA_FLAG_AUTONOMOUS)) {
      check_prefix(&dag->prefix_info, NULL);
    }

    remove_parents(dag, 0);
  }
  dag->used = 0;
}
/*---------------------------------------------------------------------------*/
rpl_parent_t *
rpl_add_parent(rpl_dag_t *dag, rpl_dio_t *dio, uip_ipaddr_t *addr)
{
  rpl_parent_t *p = NULL;
  /* Is the parent known by ds6? Drop this request if not.
   * Typically, the parent is added upon receiving a DIO. */
  uip_lladdr_t *lladdr = uip_ds6_nbr_lladdr_from_ipaddr(addr);

//  PRINTF("RPL: rpl_add_parent lladdr %p\n", lladdr);
  if(lladdr != NULL) {
    /* Add parent in rpl_parents */
    p = nbr_table_add_lladdr(rpl_parents, (rimeaddr_t *)lladdr);
    p->dag = dag;
    p->rank = dio->rank;
    p->dtsn = dio->dtsn;
    p->rssi= cc2420_last_rssi-45;
    p->update_time = clock_seconds();
if(dag->rank > p->rank)
    p->flag= PRNT;
else
   p->flag=NBR;
    p->link_metric = RPL_INIT_LINK_METRIC * RPL_DAG_MC_ETX_DIVISOR;

#if RPL_DAG_MC != RPL_DAG_MC_NONE
    memcpy(&p->mc, &dio->mc, sizeof(p->mc));
#endif /* RPL_DAG_MC != RPL_DAG_MC_NONE */
  }

  return p;
}
/*---------------------------------------------------------------------------*/
static rpl_parent_t *
find_parent_any_dag_any_instance(uip_ipaddr_t *addr)
{
  uip_ds6_nbr_t *ds6_nbr = uip_ds6_nbr_lookup(addr);
  uip_lladdr_t *lladdr = uip_ds6_nbr_get_ll(ds6_nbr);
  return nbr_table_get_from_lladdr(rpl_parents, (rimeaddr_t *)lladdr);
}
/*---------------------------------------------------------------------------*/
rpl_parent_t *
rpl_find_parent(rpl_dag_t *dag, uip_ipaddr_t *addr)
{
  rpl_parent_t *p = find_parent_any_dag_any_instance(addr);
  if(p != NULL && p->dag == dag) {
    return p;
  } else {
    return NULL;
  }
}
/*---------------------------------------------------------------------------*/
static rpl_dag_t *
find_parent_dag(rpl_instance_t *instance, uip_ipaddr_t *addr)
{
  rpl_parent_t *p = find_parent_any_dag_any_instance(addr);
  if(p != NULL) {
    return p->dag;
  } else {
    return NULL;
  }
}
/*---------------------------------------------------------------------------*/
rpl_parent_t *
rpl_find_parent_any_dag(rpl_instance_t *instance, uip_ipaddr_t *addr)
{
  rpl_parent_t *p = find_parent_any_dag_any_instance(addr);
  if(p && p->dag && p->dag->instance == instance) {
    return p;
  } else {
    return NULL;
  }
}
/*---------------------------------------------------------------------------*/
rpl_dag_t *
rpl_select_dag(rpl_instance_t *instance, rpl_parent_t *p)
{
  rpl_parent_t *last_parent;
  rpl_dag_t *dag, *end, *best_dag;
  rpl_rank_t old_rank;

  old_rank = instance->current_dag->rank;
  last_parent = instance->current_dag->preferred_parent;

  best_dag = instance->current_dag;
  if(best_dag->rank != ROOT_RANK(instance)) {
    if(rpl_select_parent(p->dag) != NULL) {
      if(p->dag != best_dag) {
        best_dag = instance->of->best_dag(best_dag, p->dag);
      }
    } else if(p->dag == best_dag) {
      best_dag = NULL;
      for(dag = &instance->dag_table[0], end = dag + RPL_MAX_DAG_PER_INSTANCE; dag < end; ++dag) {
        if(dag->used && dag->preferred_parent != NULL && dag->preferred_parent->rank != INFINITE_RANK) {
          if(best_dag == NULL) {
            best_dag = dag;
          } else {
            best_dag = instance->of->best_dag(best_dag, dag);
          }
        }
      }
    }
  }

  if(best_dag == NULL) {
    /* No parent found: the calling function handle this problem. */
    return NULL;
  }

  if(instance->current_dag != best_dag) {
    /* Remove routes installed by DAOs. */
    rpl_remove_routes(instance->current_dag);

//    PRINTF("RPL: New preferred DAG: ");
//    PRINT6ADDR(&best_dag->dag_id);
//    PRINTF("\n");

    if(best_dag->prefix_info.flags & UIP_ND6_RA_FLAG_AUTONOMOUS) {
      check_prefix(&instance->current_dag->prefix_info, &best_dag->prefix_info);
    } else if(instance->current_dag->prefix_info.flags & UIP_ND6_RA_FLAG_AUTONOMOUS) {
      check_prefix(&instance->current_dag->prefix_info, NULL);
    }

    best_dag->joined = 1;
    instance->current_dag->joined = 0;
    instance->current_dag = best_dag;
  }

  instance->of->update_metric_container(instance);
  /* Update the DAG rank. */
  best_dag->rank = instance->of->calculate_rank(best_dag->preferred_parent, 0);
  if(last_parent == NULL || best_dag->rank < best_dag->min_rank) {
    best_dag->min_rank = best_dag->rank;
  } else if(!acceptable_rank(best_dag, best_dag->rank)) {
 //   PRINTF("RPL: New rank unacceptable!\n");
    rpl_set_preferred_parent(instance->current_dag, NULL);
    if(instance->mop != RPL_MOP_NO_DOWNWARD_ROUTES && last_parent != NULL) {
      /* Send a No-Path DAO to the removed preferred parent. */
      dao_output(last_parent, RPL_ZERO_LIFETIME);
    }
    return NULL;
  }

  if(best_dag->preferred_parent != last_parent) {
    rpl_set_default_route(instance, rpl_get_parent_ipaddr(best_dag->preferred_parent));
//    PRINTF("RPL: Changed preferred parent, rank changed from %u to %u\n",(unsigned)old_rank, best_dag->rank);
    RPL_STAT(rpl_stats.parent_switch++);
    if(instance->mop != RPL_MOP_NO_DOWNWARD_ROUTES) {
      if(last_parent != NULL) {
        /* Send a No-Path DAO to the removed preferred parent. */
        dao_output(last_parent, RPL_ZERO_LIFETIME);
      }
      /* The DAO parent set changed - schedule a DAO transmission. */
      RPL_LOLLIPOP_INCREMENT(instance->dtsn_out);
      rpl_schedule_dao(instance);
    }
    rpl_reset_dio_timer(instance);
  } else if(best_dag->rank != old_rank) {
//    PRINTF("RPL: Preferred parent update, rank changed from %u to %u\n",(unsigned)old_rank, best_dag->rank);
  }
  return best_dag;
}
/*---------------------------------------------------------------------------*/
rpl_parent_t *
rpl_select_parent(rpl_dag_t *dag)
{
  rpl_parent_t *p, *best;

  best = NULL;

  p = nbr_table_head(rpl_parents);
  while(p != NULL) {
    if(p->rank == INFINITE_RANK) {
      /* ignore this neighbor */
    } else if(best == NULL) {
      best = p;
    } else {
      best = dag->instance->of->best_parent(best, p);
    }
    p = nbr_table_next(rpl_parents, p);
  }

  if(best != NULL) {
    rpl_set_preferred_parent(dag, best);
  }

  return best;
}
/*---------------------------------------------------------------------------*/
void
rpl_remove_parent(rpl_parent_t *parent)
{
//  PRINTF("RPL: Removing parent ");
//  PRINT6ADDR(rpl_get_parent_ipaddr(parent));
//  PRINTF("\n");

  rpl_nullify_parent(parent);

  nbr_table_remove(rpl_parents, parent);
}
/*---------------------------------------------------------------------------*/
void
rpl_nullify_parent(rpl_parent_t *parent)
{
  rpl_dag_t *dag = parent->dag;
  /* This function can be called when the preferred parent is NULL, so we
     need to handle this condition in order to trigger uip_ds6_defrt_rm. */
  if(parent == dag->preferred_parent || dag->preferred_parent == NULL) {
    rpl_set_preferred_parent(dag, NULL);
    dag->rank = INFINITE_RANK;
    if(dag->joined) {
      if(dag->instance->def_route != NULL) {
//        PRINTF("RPL: Removing default route ");
//        PRINT6ADDR(rpl_get_parent_ipaddr(parent));
//        PRINTF("\n");
        uip_ds6_defrt_rm(dag->instance->def_route);
        dag->instance->def_route = NULL;
      }
      dao_output(parent, RPL_ZERO_LIFETIME);
    }
  }

//  PRINTF("RPL: Nullifying parent ");
//  PRINT6ADDR(rpl_get_parent_ipaddr(parent));
//  PRINTF("\n");
}
/*---------------------------------------------------------------------------*/
void
rpl_move_parent(rpl_dag_t *dag_src, rpl_dag_t *dag_dst, rpl_parent_t *parent)
{
  if(parent == dag_src->preferred_parent) {
      rpl_set_preferred_parent(dag_src, NULL);
      dag_src->rank = INFINITE_RANK;
    if(dag_src->joined && dag_src->instance->def_route != NULL) {
//      PRINTF("RPL: Removing default route ");
//      PRINT6ADDR(rpl_get_parent_ipaddr(parent));
//      PRINTF("\n");
//      PRINTF("rpl_move_parent\n");
      uip_ds6_defrt_rm(dag_src->instance->def_route);
      dag_src->instance->def_route = NULL;
    }
  } else if(dag_src->joined) {
    /* Remove uIPv6 routes that have this parent as the next hop. */
    rpl_remove_routes_by_nexthop(rpl_get_parent_ipaddr(parent), dag_src);
  }

//  PRINTF("RPL: Moving parent ");
//  PRINT6ADDR(rpl_get_parent_ipaddr(parent));
//  PRINTF("\n");

  list_remove(dag_src->parents, parent);
  parent->dag = dag_dst;
  list_add(dag_dst->parents, parent);
}
/*---------------------------------------------------------------------------*/
rpl_dag_t *
rpl_get_any_dag(void)
{
  int i;

  for(i = 0; i < RPL_MAX_INSTANCES; ++i) {
    if(instance_table[i].used && instance_table[i].current_dag->joined) {
      return instance_table[i].current_dag;
    }
  }
  return NULL;
}
/*---------------------------------------------------------------------------*/
rpl_instance_t *
rpl_get_instance(uint8_t instance_id)
{
  int i;

  for(i = 0; i < RPL_MAX_INSTANCES; ++i) {
    if(instance_table[i].used && instance_table[i].instance_id == instance_id) {
      return &instance_table[i];
    }
  }
  return NULL;
}
/*---------------------------------------------------------------------------*/
rpl_of_t *
rpl_find_of(rpl_ocp_t ocp)
{
  unsigned int i;

  for(i = 0;
      i < sizeof(objective_functions) / sizeof(objective_functions[0]);
      i++) {
    if(objective_functions[i]->ocp == ocp) {
      return objective_functions[i];
    }
  }

  return NULL;
}
/*---------------------------------------------------------------------------*/
void
rpl_join_instance(uip_ipaddr_t *from, rpl_dio_t *dio)
{
  rpl_instance_t *instance;
  rpl_dag_t *dag;
  rpl_parent_t *p;
  rpl_of_t *of;

  dag = rpl_alloc_dag(dio->instance_id, &dio->dag_id);
  if(dag == NULL) {
//    PRINTF("RPL: Failed to allocate a DAG object!\n");
    return;
  }

  instance = dag->instance;

  p = rpl_add_parent(dag, dio, from);
//  PRINTF("RPL: Adding ");
//  PRINT6ADDR(from);
//  PRINTF(" as a parent: ");
  if(p == NULL) {
//    PRINTF("failed\n");
    instance->used = 0;
    return;
  }
  p->dtsn = dio->dtsn;
//  PRINTF("succeeded\n");

  /* Determine the objective function by using the
     objective code point of the DIO. */
  of = rpl_find_of(dio->ocp);
  if(of == NULL) {
//    PRINTF("RPL: DIO for DAG instance %u does not specify a supported OF\n",dio->instance_id);
    rpl_remove_parent(p);
    instance->used = 0;
    return;
  }

  /* Autoconfigure an address if this node does not already have an address
     with this prefix. */
  if(dio->prefix_info.flags & UIP_ND6_RA_FLAG_AUTONOMOUS) {
    check_prefix(NULL, &dio->prefix_info);
  }

  dag->joined = 1;
  dag->preference = dio->preference;
  dag->grounded = dio->grounded;
  dag->version = dio->version;

  instance->of = of;
  instance->mop = dio->mop;
  instance->current_dag = dag;
  instance->dtsn_out = RPL_LOLLIPOP_INIT;

  instance->max_rankinc = dio->dag_max_rankinc;
  instance->min_hoprankinc = dio->dag_min_hoprankinc;
  instance->dio_intdoubl = dio->dag_intdoubl;
  instance->dio_intmin = dio->dag_intmin;
  instance->dio_intcurrent = instance->dio_intmin + instance->dio_intdoubl;
  instance->dio_redundancy = dio->dag_redund;
  instance->default_lifetime = dio->default_lifetime;
  instance->lifetime_unit = dio->lifetime_unit;

  memcpy(&dag->dag_id, &dio->dag_id, sizeof(dio->dag_id));

  /* Copy prefix information from the DIO into the DAG object. */
  memcpy(&dag->prefix_info, &dio->prefix_info, sizeof(rpl_prefix_t));

  rpl_set_preferred_parent(dag, p);
  instance->of->update_metric_container(instance);
  dag->rank = instance->of->calculate_rank(p, 0);
  /* So far this is the lowest rank we are aware of. */
  dag->min_rank = dag->rank;

  if(default_instance == NULL) {
    default_instance = instance;
  }

//  PRINTF("RPL: Joined DAG with instance ID %u, rank %hu, DAG ID ", dio->instance_id, dag->rank);
//  PRINT6ADDR(&dag->dag_id);
//  PRINTF("\n");

  ANNOTATE("#A join=%u\n", dag->dag_id.u8[sizeof(dag->dag_id) - 1]);

  rpl_reset_dio_timer(instance);
  rpl_set_default_route(instance, from);

  if(instance->mop != RPL_MOP_NO_DOWNWARD_ROUTES) {
    rpl_schedule_dao(instance);
  } else {
//    PRINTF("RPL: The DIO does not meet the prerequisites for sending a DAO\n");
  }
}

/*---------------------------------------------------------------------------*/
void
rpl_add_dag(uip_ipaddr_t *from, rpl_dio_t *dio)
{
  rpl_instance_t *instance;
  rpl_dag_t *dag, *previous_dag;
  rpl_parent_t *p;
  rpl_of_t *of;

  dag = rpl_alloc_dag(dio->instance_id, &dio->dag_id);
  if(dag == NULL) {
//    PRINTF("RPL: Failed to allocate a DAG object!\n");
    return;
  }

  instance = dag->instance;

  previous_dag = find_parent_dag(instance, from);
  if(previous_dag == NULL) {
//    PRINTF("RPL: Adding ");
//    PRINT6ADDR(from);
 //   PRINTF(" as a parent: ");
    p = rpl_add_parent(dag, dio, from);
    if(p == NULL) {
//      PRINTF("failed\n");
      dag->used = 0;
      return;
    }
//    PRINTF("succeeded\n");
  } else {
    p = rpl_find_parent(previous_dag, from);
    if(p != NULL) {
      rpl_move_parent(previous_dag, dag, p);
    }
  }

  /* Determine the objective function by using the
     objective code point of the DIO. */
  of = rpl_find_of(dio->ocp);
  if(of != instance->of ||
     instance->mop != dio->mop ||
     instance->max_rankinc != dio->dag_max_rankinc ||
     instance->min_hoprankinc != dio->dag_min_hoprankinc ||
     instance->dio_intdoubl != dio->dag_intdoubl ||
     instance->dio_intmin != dio->dag_intmin ||
     instance->dio_redundancy != dio->dag_redund ||
     instance->default_lifetime != dio->default_lifetime ||
     instance->lifetime_unit != dio->lifetime_unit) {
//    PRINTF("RPL: DIO for DAG instance %u uncompatible with previos DIO\n", dio->instance_id);
    rpl_remove_parent(p);
    dag->used = 0;
    return;
  }

  dag->used = 1;
  dag->grounded = dio->grounded;
  dag->preference = dio->preference;
  dag->version = dio->version;

  memcpy(&dag->dag_id, &dio->dag_id, sizeof(dio->dag_id));

  /* copy prefix information into the dag */
  memcpy(&dag->prefix_info, &dio->prefix_info, sizeof(rpl_prefix_t));

  rpl_set_preferred_parent(dag, p);
  dag->rank = instance->of->calculate_rank(p, 0);
  dag->min_rank = dag->rank; /* So far this is the lowest rank we know of. */

//  PRINTF("RPL: Joined DAG with instance ID %u, rank %hu, DAG ID ", dio->instance_id, dag->rank);
//  PRINT6ADDR(&dag->dag_id);
//  PRINTF("\n");

  ANNOTATE("#A join=%u\n", dag->dag_id.u8[sizeof(dag->dag_id) - 1]);

  rpl_process_parent_event(instance, p);
  p->dtsn = dio->dtsn;
}

/*---------------------------------------------------------------------------*/
static void
global_repair(uip_ipaddr_t *from, rpl_dag_t *dag, rpl_dio_t *dio)
{
  rpl_parent_t *p;

  remove_parents(dag, 0);
  dag->version = dio->version;
  dag->instance->of->reset(dag);
  dag->min_rank = INFINITE_RANK;
  RPL_LOLLIPOP_INCREMENT(dag->instance->dtsn_out);

  p = rpl_add_parent(dag, dio, from);
  if(p == NULL) {
//    PRINTF("RPL: Failed to add a parent during the global repair\n");
    dag->rank = INFINITE_RANK;
  } else {
    dag->rank = dag->instance->of->calculate_rank(p, 0);
    dag->min_rank = dag->rank;
//    PRINTF("RPL: rpl_process_parent_event global repair\n");
    rpl_process_parent_event(dag->instance, p);
  }

//  PRINTF("RPL: Participating in a global repair (version=%u, rank=%hu)\n", dag->version, dag->rank);

  RPL_STAT(rpl_stats.global_repairs++);
}
/*---------------------------------------------------------------------------*/
void
rpl_local_repair(rpl_instance_t *instance)
{
  int i;

  if(instance == NULL) {
//    PRINTF("RPL: local repair requested for instance NULL\n");
    return;
  }
//  PRINTF("RPL: Starting a local instance repair\n");
  for(i = 0; i < RPL_MAX_DAG_PER_INSTANCE; i++) {
    if(instance->dag_table[i].used) {
      instance->dag_table[i].rank = INFINITE_RANK;
      nullify_parents(&instance->dag_table[i], 0);
    }
  }

  rpl_reset_dio_timer(instance);

  RPL_STAT(rpl_stats.local_repairs++);
}
/*---------------------------------------------------------------------------*/
void
rpl_recalculate_ranks(void)
{
  rpl_parent_t *p;
//uip_ipaddr_t *x;
 //rpl_parent_t *pref_parent =(&instance_table[0])->current_dag->preferred_parent;//mark
  /*
   * We recalculate ranks when we receive feedback from the system rather
   * than RPL protocol messages. This periodical recalculation is called
   * from a timer in order to keep the stack depth reasonably low.
   */
  p = nbr_table_head(rpl_parents);

//printf("rank= %u:{", (((&instance_table[0])->current_dag->rank)/256));//,list_length((&instance_table[0])->current_dag->parents));//256);//mark


  while(p != NULL) {
	/*x=rpl_get_parent_ipaddr(p);
	printf("(");//mark
	if(p==pref_parent)//mark
		printf("pref-prnt ");//mark
	printf(" %02x%02x ",((uint8_t *)x)[14], ((uint8_t *)x)[15]);//mark
	printf("etx=%d.%d, rssi=%d, rank=%u", p->link_metric/128, p->link_metric%128 ,p->rssi, p->rank/256);//mark*/
	/* if(p->rssi > RSSI_THRESHOLD && (clock_seconds()-p->update_time)<AGE_THRESHOLD && p->flag==PRNT)
		{p->flag= Pt_PRNT;
		PRINTF("P_t prnt=");
		PRINT6ADDR(rpl_get_parent_ipaddr(p));
		PRINTF(",");
		}*/


    if(p->dag != NULL && p->dag->instance && p->updated) {
      p->updated = 0;
//      PRINTF("RPL: rpl_process_parent_event recalculate_ranks\n");
      if(!rpl_process_parent_event(p->dag->instance, p)) {
//        PRINTF("RPL: A parent was dropped\n");
      }
	
    }
//printf(")");//mark
    p = nbr_table_next(rpl_parents, p);
  }
//printf("}\n");//mark
}
/*---------------------------------------------------------------------------*/
int
rpl_process_parent_event(rpl_instance_t *instance, rpl_parent_t *p)
{
  int return_value;

#if DEBUG
  rpl_rank_t old_rank;
  old_rank = instance->current_dag->rank;
#endif /* DEBUG */

  return_value = 1;

  if(!acceptable_rank(p->dag, p->rank)) {
    /* The candidate parent is no longer valid: the rank increase resulting
       from the choice of it as a parent would be too high. */
//    PRINTF("RPL: Unacceptable rank %u\n", (unsigned)p->rank);
    rpl_nullify_parent(p);
    if(p != instance->current_dag->preferred_parent) {
      return 0;
    } else {
      return_value = 0;
    }
  }

  if(rpl_select_dag(instance, p) == NULL) {
    /* No suitable parent; trigger a local repair. */
//    PRINTF("RPL: No parents found in any DAG\n");
    rpl_local_repair(instance);
    return 0;
  }

#if DEBUG
  if(DAG_RANK(old_rank, instance) != DAG_RANK(instance->current_dag->rank, instance)) {
 //   PRINTF("RPL: Moving in the instance from rank %hu to %hu\n",DAG_RANK(old_rank, instance), DAG_RANK(instance->current_dag->rank, instance));
    if(instance->current_dag->rank != INFINITE_RANK) {
//      PRINTF("RPL: The preferred parent is ");
//      PRINT6ADDR(rpl_get_parent_ipaddr(instance->current_dag->preferred_parent));
//      PRINTF(" (rank %u)\n",(unsigned)DAG_RANK(instance->current_dag->preferred_parent->rank, instance));
    } else {
//      PRINTF("RPL: We don't have any parent");
    }
  }
#endif /* DEBUG */

  return return_value;
}
/*---------------------------------------------------------------------------*/
void
rpl_process_dio(uip_ipaddr_t *from, rpl_dio_t *dio)
{
  rpl_instance_t *instance;
  rpl_dag_t *dag, *previous_dag;
  rpl_parent_t *p;

  if(dio->mop != RPL_MOP_DEFAULT) {
//    PRINTF("RPL: Ignoring a DIO with an unsupported MOP: %d\n", dio->mop);
    return;
  }

  dag = get_dag(dio->instance_id, &dio->dag_id);
  instance = rpl_get_instance(dio->instance_id);

  if(dag != NULL && instance != NULL) {
    if(lollipop_greater_than(dio->version, dag->version)) {
      if(dag->rank == ROOT_RANK(instance)) {
//	PRINTF("RPL: Root received inconsistent DIO version number\n");
	dag->version = dio->version;
	RPL_LOLLIPOP_INCREMENT(dag->version);
	rpl_reset_dio_timer(instance);
      } else {
//        PRINTF("RPL: Global Repair\n");
        if(dio->prefix_info.length != 0) {
          if(dio->prefix_info.flags & UIP_ND6_RA_FLAG_AUTONOMOUS) {
//            PRINTF("RPL : Prefix announced in DIO\n");
            rpl_set_prefix(dag, &dio->prefix_info.prefix, dio->prefix_info.length);
          }
        }
	global_repair(from, dag, dio);
      }
      return;
    }

    if(lollipop_greater_than(dag->version, dio->version)) {
      /* The DIO sender is on an older version of the DAG. */
//      PRINTF("RPL: old version received => inconsistency detected\n");
      if(dag->joined) {
        rpl_reset_dio_timer(instance);
        return;
      }
    }
  }

  if(instance == NULL) {
//    PRINTF("RPL: New instance detected: Joining...\n");
    rpl_join_instance(from, dio);
    return;
  }

  if(dag == NULL) {
//    PRINTF("RPL: Adding new DAG to known instance.\n");
    rpl_add_dag(from, dio);
    return;
  }


  if(dio->rank < ROOT_RANK(instance)) {
//    PRINTF("RPL: Ignoring DIO with too low rank: %u\n", (unsigned)dio->rank);
    return;
  } else if(dio->rank == INFINITE_RANK && dag->joined) {
    rpl_reset_dio_timer(instance);
  }
  
  /* Prefix Information Option treated to add new prefix */
  if(dio->prefix_info.length != 0) {
    if(dio->prefix_info.flags & UIP_ND6_RA_FLAG_AUTONOMOUS) {
//      PRINTF("RPL : Prefix announced in DIO\n");
      rpl_set_prefix(dag, &dio->prefix_info.prefix, dio->prefix_info.length);
    }
  }

  if(dag->rank == ROOT_RANK(instance)) {
    if(dio->rank != INFINITE_RANK) {
      instance->dio_counter++;
    }
    return;
  }

  /*
   * At this point, we know that this DIO pertains to a DAG that
   * we are already part of. We consider the sender of the DIO to be
   * a candidate parent, and let rpl_process_parent_event decide
   * whether to keep it in the set.
   */

  p = rpl_find_parent(dag, from);

if (p!=NULL)
	{
		p->rssi= moving_average(p->rssi, cc2420_last_rssi-45);
    		p->update_time = clock_seconds();
		if (p->rank < dag->rank && p->flag==NBR)
    			p->flag=PRNT;
		if (p->rank >= dag->rank && p->flag==PRNT)
			p->flag=NBR;
		//PRINTF("process_dio-update prnt: p->update_time=%lu,p->rssi=%d \n",p->update_time, p->rssi);
		//PRINTF("ID=%02x%02x \n",((uint8_t *)from)[14], ((uint8_t *)from)[15]);
	}

  if(p == NULL) {
    previous_dag = find_parent_dag(instance, from);
    if(previous_dag == NULL) {
      /* Add the DIO sender as a candidate parent. */
      p = rpl_add_parent(dag, dio, from);
      if(p == NULL) {
//        PRINTF("RPL: Failed to add a new parent (");
//        PRINT6ADDR(from);
//        PRINTF(")\n");
        return;
      }
//      PRINTF("RPL: New candidate parent with rank %u: ", (unsigned)p->rank);
//      PRINT6ADDR(from);
//      PRINTF("\n");
    } else {
      p = rpl_find_parent(previous_dag, from);
      if(p != NULL) {
        rpl_move_parent(previous_dag, dag, p);
      }
    }
  } else {
    if(p->rank == dio->rank) {
//      PRINTF("RPL: Received consistent DIO\n");
      if(dag->joined) {
        instance->dio_counter++;
      }
    } else {
      p->rank=dio->rank;
    }
  }

//  PRINTF("RPL: preferred DAG ");
//  PRINT6ADDR(&instance->current_dag->dag_id);
//  PRINTF(", rank %u, min_rank %u, ",instance->current_dag->rank, instance->current_dag->min_rank);
//  PRINTF("parent rank %u, parent etx %u, link metric %u, instance etx %u\n",p->rank, -1/*p->mc.obj.etx*/, p->link_metric, instance->mc.obj.etx);

  /* We have allocated a candidate parent; process the DIO further. */

#if RPL_DAG_MC != RPL_DAG_MC_NONE
  memcpy(&p->mc, &dio->mc, sizeof(p->mc));
#endif /* RPL_DAG_MC != RPL_DAG_MC_NONE */
  if(rpl_process_parent_event(instance, p) == 0) {
//    PRINTF("RPL: The candidate parent is rejected\n");
    return;
  }

  /* We don't use route control, so we can have only one official parent. */
  if(dag->joined && p == dag->preferred_parent) {
    if(should_send_dao(instance, dio, p)) {
      RPL_LOLLIPOP_INCREMENT(instance->dtsn_out);
      rpl_schedule_dao(instance);
    }
    /* We received a new DIO from our preferred parent.
     * Call uip_ds6_defrt_add to set a fresh value for the lifetime counter */
    uip_ds6_defrt_add(from, RPL_LIFETIME(instance, instance->default_lifetime));
  }
  p->dtsn = dio->dtsn;
}
/*---------------------------------------------------------------------------*/

char find_pref(void)
{
	 static struct etimer test_one_tx;

	int timer;
	char pt_exist;
	pt_exist = rpl_pt_parents();
//	int i = 4;
//	int wait = PROBE_NUM_THRESHOLD * (PROBE_INTERVAL+1);
//	 etimer_set(&test_one_tx, wait*CLOCK_SECOND);
	if(pt_exist==1)
	{

//		cc2420_set_txpower(Tx_array[i]);

//		printf("Tx=%d\n", cc2420_get_txpower());

		timer = (PROBE_INTERVAL * CLOCK_SECOND);
		ctimer_set(&probe_timer, timer, &handle_probe_timer, pt_parents);
	}
	return pt_exist;
}

/*---------------------------------------------------------------------------*/


char rpl_pt_parents(void)
{	
	char pt_exist;
	rpl_parent_t *p;
	struct ip_addr_list_struct test;
	uip_ipaddr_t *dest;
	unsigned long age;
rpl_rank_t srank;
	rpl_parent_t * pref_parent;

/*if(flag==0)
{
	flag=1;
	memb_init(&pt_prnt_mem);
	list_init(ip_addr_list);
	
}*/
pt_parents[0].rank=INFINITE_RANK;
pt_parents[0].ipaddr=NULL;
pt_parents[1]=pt_parents[0];
pt_parents[2]=pt_parents[1];


//printf("\nP_T={");

//printf("rank= %u:{", (((&instance_table[0])->current_dag->rank)/256));
pref_parent =(&instance_table[0])->current_dag->preferred_parent;
p = nbr_table_head(rpl_parents);
srank = p->dag->rank;
  	for( ;p != NULL;p = nbr_table_next(rpl_parents, p)) 
	{	
				if(p==pref_parent)
				{continue;}
				dest=rpl_get_parent_ipaddr(p);
				age = (clock_seconds()-(p->update_time))/2000;
//				 printf("prnt rssi=%d,rank=%u\n",p->rssi, p->rank);
				if(p->rssi > RSSI_THRESHOLD && (p->rank)<srank)// age<AGE_THRESHOLD &&
				{	
					p->numtx = 0;
					p->recv = 0;
					if(p->rank < pt_parents[0].rank)
						{	
							test=pt_parents[1];
							pt_parents[1]=pt_parents[0];
							pt_parents[2]=test;
							pt_parents[0].rank=p->rank;
							pt_parents[0].ipaddr=dest;
							continue;	
				
						}
					if(p->rank < pt_parents[1].rank)
						{	
							pt_parents[2]=pt_parents[1];
							pt_parents[1].rank=p->rank;
							pt_parents[1].ipaddr=dest;
							continue;
						}
					if(p->rank < pt_parents[2].rank)
						{	
							pt_parents[2].rank= p->rank;
							pt_parents[2].ipaddr= dest;
						}
				}
		
	}
		
	
printf("P_T={%02x, %02x, %02x}\n",((uint8_t *)pt_parents[0].ipaddr)[15],((uint8_t *)pt_parents[1].ipaddr)[15],((uint8_t *)pt_parents[2].ipaddr)[15]);
if(pt_parents[0].rank!=INFINITE_RANK)
{
	 pt_exist = 1;

}
else 
{	
	//printf("there is no pt_pref_prnt\n");
	 pt_exist = 0;
}
return  pt_exist;
	
}
/* -------------------------------------------------------------------------------*/
static void handle_probe_timer(void *pt)
{	struct ip_addr_list_struct *ptr;
	uip_ipaddr_t *dest;
	ptr =(struct ip_addr_list_struct *)pt;	
	//ptr =(list_t)pt;	
	index=0;
	int rand_time=1*CLOCK_SECOND;


	
//printf("probe timer handler\n");
	//for(i=0; ptr[i].rank!=INFINITE_RANK && i<3; i++)
	//{	
	if (ptr[0].rank==INFINITE_RANK)
		return;
	dest=ptr[index].ipaddr;

	ctimer_set(&stagger_timer, rand_time, &handle_stagger_timer, dest);
	probe_number ++;
	if(probe_number<PROBE_NUM_THRESHOLD)
	{     ctimer_reset(&probe_timer);
	}
	else 
	{       
		
		return;
	}
}
/*--------------------------------------------------------------------------------*/
void handle_find_pref_timer(void * ptr)
{	int i ;
	uip_ipaddr_t *dest;
	rpl_parent_t *p;
	rpl_dag_t *dag=(&instance_table[0])->current_dag;
	for(i=0; i<3 ; i++)
	{
		if(pt_parents[i].rank !=INFINITE_RANK )
		{
			dest=pt_parents[i].ipaddr;
//			if(dest==NULL){printf("null dest\n");}

			p=rpl_find_parent(dag, dest);
			printf("new ETX=%d, numtx=%d , recv=%d\n", p->link_metric , p->numtx, p->recv);

		}


	}
}


/*--------------------------------------------------------------------------------*/
static void handle_stagger_timer(void * ptr)
{
int rand_time;
uip_ipaddr_t *dest;
dest=(uip_ipaddr_t *)ptr;

probe_output(dest);
//printf("probe %d ADDR=%02x%02x \n",probe_number,((uint8_t *)dest)[14], ((uint8_t *)dest)[15]);
probe_total++;

//printf("probe %d ADDR=%02x%02x \n",probe_number,((uint8_t *)dest)[14], ((uint8_t *)dest)[15]);
//printf("UNJA");

//printf("stagger-rand_timer=%d", rand_time);
rand_time=(random_rand()*CLOCK_SECOND)*2/RANDOM_RAND_MAX;
//rand_time=1*CLOCK_SECOND;
index++;
if (index<3 && pt_parents[index].rank !=INFINITE_RANK )
	{
	dest=pt_parents[index].ipaddr; 
	ctimer_set(&stagger_timer, rand_time, &handle_stagger_timer, dest);
	}
else
	{	
		if(probe_number==(PROBE_NUM_THRESHOLD))

		{	probe_number=0;
			adjust_ETX_th();
			printf("\nprobe_total=%d\n",probe_total);
			ctimer_set(&find_pref_timer, 5*CLOCK_SECOND, &handle_find_pref_timer, NULL);

		}
	}

}

/*--------------------------------------------------------------------------------*/
void monitor_parents(void)
{
	rpl_parent_t *p;
	uip_ipaddr_t *dest;
 uint16_t temp1,temp2;
	rpl_parent_t *pref_parent =(&instance_table[0])->current_dag->preferred_parent;
//	rpl_instance_t *instance=&instance_table[0];
temp1 = ((((&instance_table[0])->current_dag->rank)%256)*100)/256;
	printf("rank= %u.%u:{", (((&instance_table[0])->current_dag->rank)/256), temp1);
	for(p = nbr_table_head(rpl_parents); p != NULL ;     p = nbr_table_next(rpl_parents, p))
	{	dest=rpl_get_parent_ipaddr(p);
		printf("(");
		if(p==pref_parent)
			printf("pref-prnt ");
		printf("%02x ", ((uint8_t *)dest)[15]);
temp1 = ((p->link_metric%128)*100)/128;
temp2 = ((p->rank%256)*100)/256;
		printf("etx=%d.%2d,rank=%u.%2u,%d)", p->link_metric/128, temp1 ,p->rank/256 , temp2, p->rssi);
		  /*PRINTF("RPL: My path ETX to the root is %u.%u\n",
			instance->mc.obj.etx / RPL_DAG_MC_ETX_DIVISOR,
			(instance->mc.obj.etx % RPL_DAG_MC_ETX_DIVISOR * 100) /
			 RPL_DAG_MC_ETX_DIVISOR);*/
		}
	printf("}\n");

}

#endif /* UIP_CONF_IPV6 */