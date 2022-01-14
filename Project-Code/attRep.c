/*
 * Copyright (C) 2014 Freek van Tienen
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file attRep.c
 *  @brief Implementation of the attraction repulsion algorithm in paparazzi.
 *  @details This implementation of the attraction repulsion algorithm was created
 *  by OVGU Members for the SwarmLab project to research swarm behaviours of
 *  quadcopters in simulated environment.
 */

#include "swarm/swarm.h"
#include "swarm/swarm_info.h"
#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "subsystems/datalink/telemetry.h"
#include "math.h"

#include "subsystems/navigation/waypoints.h"
#include "state.h"


/**
 * @brief radius of the simulated globe
 */
#ifndef GLOBE_RADIUS
#define GLOBE_RADIUS 6371000
#endif

/**
 * @brief follow offset in X direction
 * @details FOLLOW_OFFSET_ X and Y are all in ENU frame  
 */
#ifndef FOLLOW_OFFSET_X
#define FOLLOW_OFFSET_X 0.0f
#endif

/**
 * @brief follow offset in Y direction
 * @details FOLLOW_OFFSET_ X and Y are all in ENU frame  
 */
#ifndef FOLLOW_OFFSET_Y
#define FOLLOW_OFFSET_Y 0.0f
#endif

/**
 * @brief defines flight height of copters
 * @details defined as altitude above ground 
 */
#ifndef FLIGHT_HEIGHT
#define FLIGHT_HEIGHT 47.0f
#endif

/**
 * @brief defines gravity in simulation
 * @details sets (g)ravity value
 */
#ifndef GRAVITY
#define GRAVITY 1.985f
#endif

/**
 * @brief defines comfy distance for copters
 * @details sets comfy (d)istance value
 */
#ifndef COMFY_DIST
#define COMFY_DIST 4.8f
#endif

/**
 * @brief defines region size for repell and attraction regions
 * @details sets (r)region size value
 */
#ifndef REGION_SIZE
#define REGION_SIZE 22.5f
#endif

/**
 * @brief defines region size for repell and attraction regions 
 * for copters
 * @details sets (d)rone(r)egion size value
 */
#ifndef DRONE_REGION_SIZE
#define DRONE_REGION_SIZE 2.4f
#endif

/**
 * @brief defines attraction multiplier value
 * @details (a)ttraction(m)multiplier parameter -> ka = g * am
 */
#ifndef ATTRECTION_MULTIPLIER
#define ATTRECTION_MULTIPLIER 4.4f
#endif

/**
 * @brief defines repulsion multiplier value
 * @details (r)epulsion(m)ultiplier parameter -> kb = g * rm
 */
#ifndef REPULSION_MULTIPLIER
#define REPULSION_MULTIPLIER 4.4f
#endif

/**
 * @brief defines copter attraction multiplier
 * @details (d)rone(a)ttraction(m)ultiplier parameter -> kad = g * dam
 */
#ifndef DRONE_ATTRECTION_MULTIPLIER
#define DRONE_ATTRECTION_MULTIPLIER 0.3f
#endif

/**
 * @brief defines copter repulsion multiplier
 * @details (d)rone(r)epulsion(m)ultiplier parameter -> kbd = g * drm
 */
#ifndef DRONE_REPULSION_MULTIPLIER
#define DRONE_REPULSION_MULTIPLIER 1.0f
#endif

/**
 * @brief defines drone velocity limit
 * @details max velocity value for copters
 */
#ifndef VELOCITY_LIMIT
#define VELOCITY_LIMIT 5.0f
#endif

/**
 * @brief defines swarm waypoint id
 */
#ifndef SWARM_WAYPOINT_ID
#error "Please define SWARM_WAYPOINT_ID with the ID of FOLLOW wp"
#endif

/**
 * @brief defines first attraction point id
 */
#ifndef FIRST_ATTRACTION_POINT_ID
#error "Please define the FIRST_ATTRACTION_POINT_ID"
#endif

/**
 * @brief defines last attraction point id
 */
#ifndef LAST_ATTRACTION_POINT_ID
#error "Please define the LAST_ATTRACTION_POINT_ID"
#endif

/**
 * @brief defines first repellpoint id
 */
#ifndef FIRST_REPELL_POINT_ID
#error "Please define the  FIRST_REPELL_POINT_ID"
#endif

/**
 * @brief defines last repellpoint id
 */
#ifndef LAST_REPELL_POINT_ID
#error "Please define the  LAST_REPELL_POINT_ID"
#endif

/**
 * @brief defines first swarm member id
 */
#ifndef FIRST_SWARM_MEMBER_ID
#error "Please define the FIRST_SWARM_MEMBER_ID"
#endif

/**
 * @brief defines last swarm member id
 */
#ifndef LAST_SWARM_MEMBER_ID
#error "Please define the LAST_SWARM_MEMBER_ID"
#endif

/*******************************************************************************************************************
 * @brief defines debug message
 * @details consists of positions, ids, attraction and repulsion forces 
 * @param own_pos EnuCord_f for own position
 * @param target_pos EnuCord_f for target position
 * @param target_ac_id target ac id
 * 
 * @param attraction_force attraction force
 * @param attraction_d attraction distance
 * @param attraction_strength attraction strength
 * @param attraction attraction bool 
 * 
 * @param repulsion_force repulsion force
 * @param repulsion_d repulsion dstance
 * @param repulsion_strength repulsion strength
 * @param repulsion repulsion bool
 ********************************************************************************************************************/
struct Message_Debug {
   struct EnuCoor_f own_pos;                  ///< @brief EnuCord_f for own position
   struct EnuCoor_f target_pos;               ///< @brief EnuCord_f for target position
   uint8_t target_ac_id;                      ///< @brief target ac id

   struct EnuCoor_f attraction_force;         ///< @brief attraction force
   float attraction_d;                        ///< @brief attraction distance
   float attraction_strength;                 ///< @brief attraction strength
   bool attraction;                           ///< @brief attraction bool 

   struct EnuCoor_f repulsion_force;          ///< @brief repulsion force
   float repulsion_d;                         ///< @brief repulsion dstance
   float repulsion_strength;                  ///< @brief repulsion strength
   bool repulsion;                            ///< @brief repulsion bool
};

/*********************************************************************************************************************
 * @brief goal message
 * @details message that contains "reached status" of each copter
 * @param wp_id id of approched waypoint
 * @param own_pos position of copter
 * @param reached bool if goal is reached or not
 *********************************************************************************************************************/
struct Message_Goal {
   uint8_t wp_id;                 ///< @brief id of approched waypoint
   struct LlaCoor_f own_pos;      ///< @brief position of copter
   bool reached;                  ///< @brief bool if goal is reached or not
};

static struct EnuCoor_f acc = {0.0f, 0.0f, 0.0f};                                                                                               ///< @brief acceleration struct
static struct Message_Debug msg = {{0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f},0,{0.0f,0.0f,0.0f},0.0f,0.0f,false,{0.0f,0.0f,0.0f},0.0f,0.0f,false};      ///< @brief debug message
static struct Message_Goal syncLink = {0,{0.0f,0.0f,0.0f},false};                                                                               ///< @brief goal message
static struct LlaCoor_f att_point = {0.0f,0.0f,0.0f};                                                                                           ///< @brief attraction point
static struct LlaCoor_f current_pos = {0.0f,0.0f,0.0f};                                                                                         ///< @brief current copter position

//LlaCoor_f = {lat,lon,alt} specified in radients in a floating point number format(as specified in the paparazzi documentation)
//LlaCoor_i = {lat,lon,alt} specified in degrees in an integer format(as specified in the paparazzi documentation)
//EnuCoor_i = {lat,lon,alt} specified in meters in an integer format(as specified in the paparazzi documentation)
//EnuCoor_f = {lat,lon,alt} specified in meters in an floating point number format(as specified in the paparazzi documentation)


/** 
 * @brief get position in local ENU coordinates (float).
 * @param ac_id aircraft id of aircraft info to get
 * @return position in EnuCoor_f
 */
static struct EnuCoor_f *getPositionEnu_f(uint8_t ac_id)
{
  return (ti_acs[ti_acs_id[ac_id]].ac_id != ac_id)? NULL: acInfoGetPositionEnu_f(ac_id);
}

/** 
 * @brief send acceleration info 
 * @param trans transport channel
 * @param dev link device
 */
static void send_acc_info(struct transport_tx *trans, struct link_device *dev) {
	pprz_msg_send_ACC(trans, dev, AC_ID, &acc.x, &acc.y, &acc.z);
}

/** 
 * @brief send goal info
 * @param trans transtransport channel
 * @param dev link device
 */
static void send_goal_info(struct transport_tx *trans, struct link_device *dev) {
	pprz_msg_send_GOAL_ACHIEVED(trans, dev, AC_ID, &syncLink.wp_id, &syncLink.own_pos.lat, &syncLink.own_pos.lon, &syncLink.own_pos.alt, (uint8_t*)&syncLink.reached);
}

/** 
 * @brief send attraction repulsion info
 * @param trans transtransport channel
 * @param dev link device
 */
static void send_attract_and_repulse_info(struct transport_tx *trans, struct link_device *dev) {
  pprz_msg_send_ATTREP(trans, dev, AC_ID, &msg.own_pos.x, &msg.own_pos.y, &msg.own_pos.z, &msg.target_pos.x, &msg.target_pos.y, &msg.target_pos.z, &msg.target_ac_id, &msg.attraction_force.x, &msg.attraction_force.y, &msg.attraction_force.z, &msg.attraction_d, &msg.attraction_strength, (uint8_t*)&msg.attraction, &msg.repulsion_force.x, &msg.repulsion_force.y, &msg.repulsion_force.z, &msg.repulsion_d, &msg.repulsion_strength, (uint8_t*)&msg.repulsion);
}

/** 
 * @brief initialize swarm
 * @details register goal, acceleration and attraction repulsion info sending
 */
void swarm_init(void) {
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GOAL_ACHIEVED, send_goal_info);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ACC, send_acc_info);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ATTREP, send_attract_and_repulse_info);
}


/** 
 * @brief gets the metric distance between two points given in the LlaCoor_f format
 * @param own_pos own position
 * @param goal_pos goal position
 */
static float getDistance(struct LlaCoor_f* own_pos, struct LlaCoor_f* goal_pos)
{
  return GLOBE_RADIUS * acos(
    sin(own_pos->lat)*sin(goal_pos->lat) + 
    cos(own_pos->lat)*cos(goal_pos->lat) * 
    cos(own_pos->lon - goal_pos->lon));
}

/** 
 * @brief converts from LlaCoor_i to LlaCoor_f by reducing to float value and switching from degrees to radients
 * @param point LlaCoor_i point thats gonna be converted to LlaCoor_f 
 * @return LlaCoor_f struct
 */
static struct LlaCoor_f toFloatPointFormat(struct LlaCoor_i* point)
{
  struct LlaCoor_f res = {0.0f,0.0f,FLIGHT_HEIGHT};
  res.lon = (((float)point->lon)/1e7)*(M_PI/180);
  res.lat = (((float)point->lat)/1e7)*(M_PI/180);
  return res;
}

/** 
 * @brief updates the content of the periodicly sent goal_achieved message
 * @param own_pos own position
 * @param att_point_id attraction point id
 */
static void updateSyncLinkMsg(struct LlaCoor_f* own_pos, uint8_t att_point_id)
{
    if(getDistance(own_pos, &att_point)<=16.25f)
    {
      syncLink.wp_id = att_point_id;
      syncLink.own_pos.lat = own_pos->lat;
      syncLink.own_pos.lon = own_pos->lon;
      syncLink.own_pos.alt = own_pos->alt;
      syncLink.reached = true;
    }
    else syncLink.reached = false;
}

/** 
 * @brief calculates attraction for copter
 * @details attraction_force = ka/distance * dist_vec
 * @param own_pos own position
 * @param pos_ac position where copter attracted to
 * @param acc accelaration
 * @param multiplier attraction multiplier
 */
static void attract(struct EnuCoor_f *own_pos, struct EnuCoor_f* pos_ac, struct EnuCoor_f* acc, float multiplier)
{
  struct EnuCoor_f force = {
  	pos_ac->x - own_pos->x,
  	pos_ac->y - own_pos->y,
  	0.0f
  };
  msg.attraction_force = force;

  float d = sqrtf(force.x*force.x + force.y*force.y);
  msg.attraction_d = d;
  msg.attraction = true;

  d = fmaxf(1.0f,d);
  float strength = GRAVITY * multiplier/d;
  msg.attraction_strength = strength;

  force.x = force.x * strength;
  force.y = force.y * strength;
  msg.attraction_force = force;

  acc->x += force.x;
  acc->y += force.y;
}

/** 
 * @brief calculated the repulsion for copter
 * @details repulsion_force = (kb*exp(-distance²/2r²)) * dist_vec
 * @param own_pos own position
 * @param pos_ac position where copter repulsed to
 * @param acc accelaration
 * @param perlimiter perlimiter
 * @param multiplier repulsion multiplier
 */
static void repulse(struct EnuCoor_f *own_pos, struct EnuCoor_f* pos_ac, struct EnuCoor_f* acc, float perlimiter, float multiplier)
{
  struct EnuCoor_f force = {
  	pos_ac->x - own_pos->x,
  	pos_ac->y - own_pos->y,
  	0.0f
  };
  msg.repulsion_force = force;

  float d = sqrtf(force.x*force.x + force.y*force.y);
  msg.repulsion_d = d;
  msg.repulsion = true;

  float strength = (GRAVITY * multiplier) * expf(-(d*d)/(2 * perlimiter * perlimiter));
  msg.repulsion_strength = strength;

  force.x = force.x * strength;
  force.y = force.y * strength;
  msg.repulsion_force = force;

  acc->x -= force.x;
  acc->y -= force.y;
}

/** 
 * @brief calculates both attraction and repulsion in combination
 * @details total_force = (kad*(distance-d)/max(distance,0.01) - kbd*exp(-distance²/(dr²/log(kbd/kad)))) * dist_vec
 * @param own_pos own position
 * @param pos_ac position where copter repulsed to
 * @param acc accelaration
 * @param reg_size region size of repell and attraction regions
 * @param comfy_dist comfy distance
 * @param att_multiplier attraction multiplier
 * @param rep_multiplier repulsion multiplier
 */
static void attRep(struct EnuCoor_f *own_pos, struct EnuCoor_f* pos_ac, struct EnuCoor_f* acc, float reg_size, float comfy_dist, float att_multiplier, float rep_multiplier)
{
    struct EnuCoor_f force = {
            pos_ac->x - own_pos->x,
            pos_ac->y - own_pos->y,
            0.0f
    };
    msg.repulsion_force = force;
    msg.attraction_force = force;

    float d = sqrtf(force.x*force.x + force.y*force.y);
    float c = (reg_size * reg_size)/logf(rep_multiplier/att_multiplier);
    msg.repulsion_d = d;
    msg.attraction_d = d;
    msg.repulsion = true;
    msg.attraction = true;

    float strength_att = (GRAVITY * att_multiplier) * ((d - comfy_dist) / fmax(d, 0.01f));
    float strength_rep = (GRAVITY * rep_multiplier) * expf(-(d*d)/c);
    msg.attraction_strength = strength_att;
    msg.repulsion_strength = strength_rep;

    force.x = force.x * (strength_att - strength_rep);
    force.y = force.y * (strength_att - strength_rep);
    msg.attraction_force = force;
    msg.repulsion_force = force;

    acc->x += force.x;
    acc->y += force.y;
}


/** 
 * @brief updates the FOLLOW_WAYPOINT_ID 
 * @details updates the FOLLOW_WAYPOINT_ID to a fixed offset from the last received location
 * of other aircraft with id FOLLOW_AC_ID
 */
void swarm_follow_wp(void)
{
  acc.x = 0.0f;
  acc.y = 0.0f;
  acc.z = 0.0f;

  struct EnuCoor_f *own_pos = stateGetPositionEnu_f();
  msg.own_pos = *own_pos;

  for(uint8_t ac_id = FIRST_SWARM_MEMBER_ID; ac_id <= LAST_SWARM_MEMBER_ID; ++ac_id)
  {
    struct EnuCoor_f *ac_pos = getPositionEnu_f(ac_id);
    if(ac_pos != NULL && ac_id != AC_ID) {
        msg.target_pos = *ac_pos;
        msg.target_ac_id = ac_id;        
        attRep(own_pos, ac_pos, &acc, DRONE_REGION_SIZE, COMFY_DIST, DRONE_ATTRECTION_MULTIPLIER, DRONE_REPULSION_MULTIPLIER);
    }
    else
    {
      msg.target_ac_id = 0;
      msg.attraction = false;
      msg.repulsion = false;
    }
  }

  for(uint8_t wp_id = FIRST_ATTRACTION_POINT_ID; wp_id < LAST_ATTRACTION_POINT_ID; ++wp_id)
  {
    struct EnuCoor_f current_att_point = { 0.0f, 0.0f, 0.0f };
    current_att_point.x = waypoint_get_x(wp_id);
    current_att_point.y = waypoint_get_y(wp_id);
    if(current_att_point.x != 0.0f && current_att_point.y != 0.0f)
      attract(own_pos, &current_att_point, &acc, ATTRECTION_MULTIPLIER);
  }

  for(uint8_t wp_id = FIRST_REPELL_POINT_ID; wp_id < LAST_REPELL_POINT_ID; ++wp_id)
  {
    struct EnuCoor_f current_rep_point = { 0.0f, 0.0f, 0.0f };
    current_rep_point.x = waypoint_get_x(wp_id);
    current_rep_point.y = waypoint_get_y(wp_id);
    if(current_rep_point.x != 0.0f && current_rep_point.y != 0.0f)
      repulse(own_pos, &current_rep_point, &acc, REGION_SIZE, REPULSION_MULTIPLIER);
  }

  struct EnuCoor_f* vel = acInfoGetVelocityEnu_f(AC_ID);
  vel->x = VELOCITY_LIMIT * tanhf(vel->x+acc.x);
  vel->y = VELOCITY_LIMIT * tanhf(vel->y+acc.y);
  acInfoSetVelocityEnu_f(AC_ID,vel);

  struct EnuCoor_i future_pos = *stateGetPositionEnu_i();
  future_pos.x += POS_BFP_OF_REAL(vel->x)+POS_BFP_OF_REAL(FOLLOW_OFFSET_X);
  future_pos.y += POS_BFP_OF_REAL(vel->y)+POS_BFP_OF_REAL(FOLLOW_OFFSET_Y);
  future_pos.z = POS_BFP_OF_REAL(FLIGHT_HEIGHT);

  // Move the waypoints
  waypoint_set_enu_i(SWARM_WAYPOINT_ID, &future_pos);
  for(uint8_t wp_id = FIRST_ATTRACTION_POINT_ID; wp_id < LAST_ATTRACTION_POINT_ID; ++wp_id)
  {
    struct LlaCoor_i* way_point = waypoint_get_lla(wp_id);
    att_point = toFloatPointFormat(way_point);
    current_pos = *stateGetPositionLla_f();
    updateSyncLinkMsg(&current_pos, wp_id);
  }
}
