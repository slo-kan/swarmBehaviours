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

/** @file follow.c
 *  @brief Follow a certain AC ID.
 * Only for rotorcraft firmware.
 */

#include "swarm/swarm.h"
#include "swarm/swarm_info.h"
#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "subsystems/datalink/telemetry.h"
#include "math.h"

#include "subsystems/navigation/waypoints.h"
#include "state.h"


/* FOLLOW_OFFSET_ X and Y are all in ENU frame */
#ifndef FOLLOW_OFFSET_X
#define FOLLOW_OFFSET_X 0.0f
#endif

#ifndef FOLLOW_OFFSET_Y
#define FOLLOW_OFFSET_Y 0.0f
#endif

// defined as altitude above ground
#ifndef FLIGHT_HEIGHT
#define FLIGHT_HEIGHT 47.0f
#endif

// maximal speed that the drone can fly
#ifndef MAX_SPEED
#define MAX_SPEED 5.0f
#endif

// radius of the globe
#ifndef GLOBE_RADIUS
#define GLOBE_RADIUS 6371000
#endif

// array initial size parameter
#ifndef ARRAY_INIT_SIZE
#define ARRAY_INIT_SIZE 16
#endif

// ndr parameter
#ifndef NUM_DIRECTION_RAYS
#define NUM_DIRECTION_RAYS 8
#endif

// G parameter
#ifndef GRAVITY
#define GRAVITY 1.985f
#endif

// g_mult parameter
#ifndef GOAL_MULT
#define GOAL_MULT 1.75f
#endif

// g_lim parameter
#ifndef GOAL_LIMIT
#define GOAL_LIMIT 30.0f
#endif

// g_sig parameter
#ifndef GOAL_SIGMA
#define GOAL_SIGMA -3.0f
#endif

// g_gam parameter
#ifndef GOAL_GAMMA
#define GOAL_GAMMA 25.0f
#endif

// d_mult parameter
#ifndef DANGER_MULT
#define DANGER_MULT 1.75f
#endif

// d_tc parameter
#ifndef DANGER_TO_CLOSE
#define DANGER_TO_CLOSE 8.0f
#endif

// d_lim parameter
#ifndef DANGER_LIMIT
#define DANGER_LIMIT 40.0f
#endif

// d_co parameter
#ifndef DANGER_CUT_OFF
#define DANGER_CUT_OFF 5.0f
#endif

// d_sig parameter
#ifndef DANGER_SIGMA
#define DANGER_SIGMA 3.85f
#endif

// d_gam parameter
#ifndef DANGER_GAMMA
#define DANGER_GAMMA 18.5f
#endif

// d_alf parameter
#ifndef DANGER_ALPHA
#define DANGER_ALPHA 20.0f
#endif

// da_mult parameter
#ifndef DRONE_MULT
#define DRONE_ATT_MULT 4.0f
#endif

// da_lim parameter
#ifndef DRONE_ATT_LIMIT
#define DRONE_ATT_LIMIT 40.0f
#endif

// da_co parameter
#ifndef DRONE_ATT_CUT_OFF
#define DRONE_ATT_CUT_OFF 18.9f
#endif

// da_sd parameter
#ifndef SWARM_DIST
#define SWARM_DIST 12.0f
#endif

// dr_dtc parameter
#ifndef DRONE_TO_CLOSE
#define DRONE_TO_CLOSE 4.0f
#endif

// dr_mult parameter
#ifndef DRONE_MULT
#define DRONE_REP_MULT 4
#endif

// dr_lim parameter
#ifndef DRONE_REP_LIMIT
#define DRONE_REP_LIMIT 30.0f
#endif

// dr_sig parameter
#ifndef DRONE_REP_SIGMA
#define DRONE_REP_SIGMA 10.0f
#endif

// dr_gam parameter
#ifndef DRONE_REP_GAMMA
#define DRONE_REP_GAMMA 1.5f
#endif

// dr_alf parameter
#ifndef DRONE_REP_ALPHA
#define DRONE_REP_ALPHA 0.1f
#endif

#ifndef SWARM_WAYPOINT_ID
#error "Please define SWARM_WAYPOINT_ID with the ID of FOLLOW wp"
#endif

#ifndef FIRST_GOAL_POINT_ID
#error "Please define the FIRST_GOAL_POINT_ID"
#endif

#ifndef LAST_GOAL_POINT_ID
#error "Please define the LAST_GOAL_POINT_ID"
#endif

#ifndef FIRST_DANGER_POINT_ID
#error "Please define the  FIRST_DANGER_POINT_ID"
#endif

#ifndef LAST_DANGER_POINT_ID
#error "Please define the  LAST_REPELL_POINT_ID"
#endif

#ifndef FIRST_SWARM_MEMBER_ID
#error "Please define the FIRST_SWARM_MEMBER_ID"
#endif

#ifndef LAST_SWARM_MEMBER_ID
#error "Please define the LAST_SWARM_MEMBER_ID"
#endif

#ifndef Pi
#define Pi 3.14159265359
#endif


typedef struct Point {
  float x;
  float y;
  float z;
} Point;


Point init_points(float x, float y, float z)
{
  Point new = {x,y,z};
  return new;
}

Point from_angle(float radians)
{ return init_points(cosf(radians),sinf(radians),0.0f); }

Point rotate2D(Point* a, float radians)
{ return init_points(cosf(radians)*a->x - sinf(radians)*a->y,cosf(radians)*a->x + sinf(radians)*a->y, a->z); }

Point sub_points(Point* a, Point* b)
{ return init_points((a->x - b->x), (a->y - b->y), (a->z - b->z)); }

Point add_points(Point* a, Point* b)
{ return init_points((a->x + b->x), (a->y + b->y), (a->z + b->z)); }

Point mult_points(Point* a, Point* b)
{ return init_points((a->x * b->x), (a->y * b->y), (a->z * b->z)); }

Point div_points(Point* a, Point* b)
{ return init_points((a->x / b->x), (a->y / b->y), (a->z / b->z)); }

Point scale_up_points(Point* a, float s)
{ return init_points((a->x * s), (a->y * s), (a->z * s)) }

Point scale_down_points(Point* a, float s)
{ return init_points((a->x / s), (a->y / s), (a->z / s)); }

float dot(Point* a, Point* b)
{ return (a->x*b->x + a->y*b->y + a->z*b->z); }

float mag(Point* a)
{ return sqrtf(a->x*a->x + a->y*a->y + a->z*a->z); }

float cosine_sim(Point* a, Point* b)
{ return dot(a,b)/(mag(a)*mag(b)); }

float dist_points(Point* a, Point* b)
{ 
  Point dist = sub_points(a,b); 
  return sqrtf(dist.x*dist.x+dist.y*dist.y+dist.z*dist.z); 
}

void set_points(Point* p, float x, float y, float z)
{ 
  if(p!=NULL) 
  {
    p->x = x; 
    p->y = y;
    p->z = z;
  }
}

void set_points(Point* p, Point* other)
{ set_points(p,other.x,other.y,other.z); }

Point strongest_force(Point* a, Point* b)
{
  if(mag(a)>mag(b)) return *a;
  return *b; 
}



struct Message_Debug {
   struct EnuCoor_f own_pos;
   struct EnuCoor_f target_pos;
   uint8_t target_ac_id;

   struct EnuCoor_f attraction_force;
   float attraction_d;
   float attraction_strength;
   bool attraction;

   struct EnuCoor_f repulsion_force;
   float repulsion_d;
   float repulsion_strength;
   bool repulsion;
};

struct Message_Goal {
   uint8_t wp_id;
   struct LlaCoor_f own_pos;  
   bool reached;
};


static float SECTOR_COS_SIM = cosf(Pi/NUM_DIRECTION_RAYS);
static Point DIRECTION_RAYS[NUM_DIRECTION_RAYS] = {{0.0f,0.0f,0.0f}};
static Point acc = {0.0f, 0.0f, 0.0f};
static Point vel = {0.0f, 0.0f, 0.0f};
static Point pos = {0.0f, 0.0f, 0.0f};
static struct Message_Debug msg = {{0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f},0,{0.0f,0.0f,0.0f},0.0f,0.0f,false,{0.0f,0.0f,0.0f},0.0f,0.0f,false};
static struct Message_Goal syncLink = {0,{0.0f,0.0f,0.0f},false};
static struct LlaCoor_f att_point = {0.0f,0.0f,0.0f};
static struct LlaCoor_f current_pos = {0.0f,0.0f,0.0f};


//LlaCoor_f = {lat,lon,alt} specified in radients in a floating point number format(as specified in the paparazzi documentation)
//LlaCoor_i = {lat,lon,alt} specified in degrees in an integer format(as specified in the paparazzi documentation)
//EnuCoor_i = {lat,lon,alt} specified in meters in an integer format(as specified in the paparazzi documentation)
//EnuCoor_f = {lat,lon,alt} specified in meters in an floating point number format(as specified in the paparazzi documentation)


static void send_acc_info(struct transport_tx *trans, struct link_device *dev) 
{	pprz_msg_send_ACC(trans, dev, AC_ID, &acc.x, &acc.y, &acc.z); }

static void send_goal_info(struct transport_tx *trans, struct link_device *dev) 
{	pprz_msg_send_GOAL_ACHIEVED(trans, dev, AC_ID, &syncLink.wp_id, &syncLink.own_pos.lat, &syncLink.own_pos.lon, &syncLink.own_pos.alt, (uint8_t*)&syncLink.reached); }

static void send_attract_and_repulse_info(struct transport_tx *trans, struct link_device *dev) 
{ pprz_msg_send_ATTREP(trans, dev, AC_ID, &msg.own_pos.x, &msg.own_pos.y, &msg.own_pos.z, &msg.target_pos.x, &msg.target_pos.y, &msg.target_pos.z, &msg.target_ac_id, &msg.attraction_force.x, &msg.attraction_force.y, &msg.attraction_force.z, &msg.attraction_d, &msg.attraction_strength, (uint8_t*)&msg.attraction, &msg.repulsion_force.x, &msg.repulsion_force.y, &msg.repulsion_force.z, &msg.repulsion_d, &msg.repulsion_strength, (uint8_t*)&msg.repulsion); }

void swarm_init(void) 
{
  for(int it=0; it<NUM_DIRECTION_RAYS; ++it)
  {
    float angle = it * (2*Pi)/NUM_DIRECTION_RAYS;
    DIRECTION_RAYS[it] = from_Angle(angle);                                                                                                                                                                                                                                                         ;
  }
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GOAL_ACHIEVED, send_goal_info);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ACC, send_acc_info);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ATTREP, send_attract_and_repulse_info);
}

//converts from LlaCoor_i to LlaCoor_f by reducing to float value and switching from degrees to radients
static struct LlaCoor_f toFloatPointFormat(struct LlaCoor_i* point)
{
  struct LlaCoor_f res = {0.0f,0.0f,FLIGHT_HEIGHT};
  res.lon = (((float)point->lon)/1e7)*(M_PI/180);
  res.lat = (((float)point->lat)/1e7)*(M_PI/180);
  return res;
}

/** Get position in local ENU coordinates (float).
 * @param[in] ac_id aircraft id of aircraft info to get
 */
static struct EnuCoor_f *getPositionEnu_f(uint8_t ac_id)
{ return (ti_acs[ti_acs_id[ac_id]].ac_id != ac_id)? NULL: acInfoGetPositionEnu_f(ac_id); }

//gets the metric distance between two points given in the LlaCoor_f format
static float getDistance(struct LlaCoor_f* own_pos, struct LlaCoor_f* goal_pos)
{
  return GLOBE_RADIUS * acos(
    sin(own_pos->lat)*sin(goal_pos->lat) + 
    cos(own_pos->lat)*cos(goal_pos->lat) * 
    cos(own_pos->lon - goal_pos->lon));
}

//updates the content of the periodicly sent goal_achieved message
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



//calculate repulsion forces to other drones
static Point linear_Repulsion(Point* pos, Point* target, float limit, float sigma, float gamma, float alpha)
{
  Point force = sub_points(target, pos);
  float magnitude = mag(&force);
  float d = fminf(magnitude, limit);
  float strength = max((sigma/(d+alpha)-gamma),0)*(-1);
  force = scale_up_points(&force,(strength/magnitude));
  return force;
}

// calculate attraction forces to other drones
static Point log_Attraction(Point* pos, Point* target, float limit, float cutOff)
{
  Point force = sub_points(target, pos);
  float magnitude = mag(&force);
  float strength;
  float d = fminf(magnitude, limit);
  //strength = GRAVITY*d-10; //simple linear
  if(d>cutOff) strength = log(d-(cutOff-GRAVITY/2))*GRAVITY; 
  else strength = 0;
  force = scale_up_points(&force,(strength/magnitude));
  return force;
}


// calculate attraction forces with GOAL_VECTORS
static Point linear_Attraction(Point* pos, Point* target, float limit, float sigma, float gamma)
{
  Point force = sub_points(target, pos);
  float magnitude = mag(&force);
  float d = fminf(magnitude, limit);
  float strength = (GRAVITY/sigma)*d+gamma;
  force = scale_up_points(&force,(strength/magnitude));
  return force;
}

// calculate repulsion force with DANGER_VECTORS
static Point limExp_Repulsion(Point* pos, Point* target, float limit, float cutOff, float sigma, float gamma, float alpha)
{
  Point force = sub_points(target, pos);
  float magnitude = mag(&force);
  float strength;
  float d = fminf(magnitude, limit);
  if (d<cutOff) strength = GRAVITY * (d/2 - limit) / sigma; //linear_rep_limit_close
  else strength = -1*expf(-1*(GRAVITY * (alpha*logf(d) - gamma)/sigma)); //exp_rep_mid_far
   force = scale_up_points(&force,(strength/magnitude));
  return force;
}


//context Steering behavior
/*
 * swarm_follow_wp(void)
 * updates the FOLLOW_WAYPOINT_ID to a fixed offset from the last received location
 * of other aircraft with id FOLLOW_AC_ID
 */
void swarm_follow_wp(void)
{
  /*
    ++++++++++++++++++++++++
    + Initialize Variables +
    ++++++++++++++++++++++++
  */
  set_points(&pos, getPositionEnu_f(AC_ID)->x, getPositionEnu_f(AC_ID)->y, getPositionEnu_f(AC_ID)->z); 
  set_points(&vel, acInfoGetVelocityEnu_f(AC_ID)->x, acInfoGetVelocityEnu_f(AC_ID)->y, acInfoGetVelocityEnu_f(AC_ID)->z);
  struct EnuCoor_f *ac_pos;
  Point danger, goal, other_pos, other_vel;
  Point alignment_force; set_points(&alignment_force, &vel);
  Point resulting_force = {0.0f,0.0f,0.0f};
  Point max_intrest_forces[NUM_DIRECTION_RAYS] = {{0.0f,0.0f,0.0f}};
  Point max_danger_forces[NUM_DIRECTION_RAYS] = {{0.0f,0.0f,0.0f}};
  Point max_member_atts[NUM_DIRECTION_RAYS] = {{0.0f,0.0f,0.0f}};
  Point max_member_reps[NUM_DIRECTION_RAYS] = {{0.0f,0.0f,0.0f}};
  Point total_forces[NUM_DIRECTION_RAYS] = {{0.0f,0.0f,0.0f}};
  bool mask[NUM_DIRECTION_RAYS] = {true};
  float member_dist, alignSim, constrainSim, rightMag, leftMag, magnitude; 
  float alignment_counter = 1.0f;
  int leftIdx,rightIdx,neighborIdx;
  int maxIdx = 0;

  
  /*
   +++++++++++++++++++++++++++
   + Context Map Preparation +
   +++++++++++++++++++++++++++
  
   create all direction segements for each map based on maximum magnitude force 
   and create a mask based on distance
  */
  for(int idx=0; idx<NUM_DIRECTION_RAYS; ++idx)
  {
    //GOAL MAP
    for(uint8_t wp_id = FIRST_GOAL_POINT_ID; wp_id < LAST_GOAL_POINT_ID; ++wp_id)
    {
      set_points(&goal,waypoint_get_x(wp_id),waypoint_get_y(wp_id),FLIGHT_HEIGHT);
      if(cosine_sim(&DIRECTION_RAYS[idx], &sub_points(&goal,&pos)) >= SECTOR_COS_SIM) 
        max_intrest_forces[idx] = strongest_force(&max_intrest_forces[idx],
                                 &linear_Attraction(&pos,&goal,GOAL_LIMIT,GOAL_SIGMA,GOAL_GAMMA));
      
    }

    //DRONE MAPS & MASK
    for(uint8_t ac_id = FIRST_SWARM_MEMBER_ID; ac_id <= LAST_SWARM_MEMBER_ID; ++ac_id)
    {
      ac_pos = getPositionEnu_f(ac_id);
      if(ac_id != AC_ID && ac_pos != NULL) 
      {
        set_points(&other_pos,ac_pos->x,ac_pos->y,ac_pos->z);
        if(cosine_sim(&DIRECTION_RAYS[idx], &sub_points(&other_pos,&pos)) >= SECTOR_COS_SIM) 
        {
          member_dist = dist_points(&other_pos,&pos);
          if(member_dist<=DRONE_TO_CLOSE) mask[idx] = false;
          else if (member_dist<=SWARM_DIST) 
          {
            set_points(&other_vel,acInfoGetVelocityEnu_f(ac_id)->x,acInfoGetVelocityEnu_f(ac_id)->y,acInfoGetVelocityEnu_f(ac_id)->z);
            alignment_force = add_points(&alignment_force,&other_vel);
            alignment_counter+=1.0f;
          }
          max_member_atts[idx] = strongest_force(&max_member_atts[idx],
                                &log_Attraction(&pos,&other_pos,DRONE_ATT_LIMIT,DRONE_ATT_CUT_OFF));
        }
        if(cosine_sim(&DIRECTION_RAYS[idx], &rotate2D(&sub_points(&other_pos,&pos),Pi)) >= SECTOR_COS_SIM)
        {
          max_member_reps[idx] = strongest_force(&max_member_reps[idx],
                                &linear_Repulsion(&pos,&other_pos,DRONE_REP_LIMIT,DRONE_REP_SIGMA,DRONE_REP_GAMMA,DRONE_REP_ALPHA)); 
        }
      }
    }

    //DANGER MAP & MASK
    for(uint8_t wp_id = FIRST_DANGER_POINT_ID; wp_id < LAST_DANGER_POINT_ID; ++wp_id)
    {
      set_points(&danger,waypoint_get_x(wp_id),waypoint_get_y(wp_id),FLIGHT_HEIGHT);
      if(cosine_sim(&DIRECTION_RAYS[idx], &rotate2D(&sub_points(&danger,&pos),Pi)) >= SECTOR_COS_SIM)
        max_danger_forces[idx] = strongest_force(&max_danger_forces[idx],
                                                 &limExp_Repulsion(&pos,&danger,DANGER_LIMIT,DANGER_CUT_OFF,DANGER_SIGMA,
                                                 DANGER_GAMMA,DANGER_ALPHA));
      else if((cosine_sim(&DIRECTION_RAYS[idx], &sub_points(&danger,&pos)) >= SECTOR_COS_SIM) && 
              (dist_points(&danger,&pos)<=DANGER_TO_CLOSE)) 
        mask[idx] = false; 
    }
  }


  /*
   ++++++++++++++++++++++++++
   + Context Map Evaluation +
   ++++++++++++++++++++++++++
  
   evaluate context steering behavior
  */

  //calculate total forces with respect to context map weightings and swarm alignment
  alignment_force = scale_down_points(&alignment_force,alignment_counter);
  for(int idx=0; idx<NUM_DIRECTION_RAYS; ++idx)
  {
    total_forces[idx] = add_points(&total_forces[idx],&scale_up_points(&max_intrest_forces[idx],GOAL_MULT));
    total_forces[idx] = add_points(&total_forces[idx],&scale_up_points(&max_member_atts[idx],DRONE_ATT_MULT));
    total_forces[idx] = add_points(&total_forces[idx],&scale_up_points(&max_member_reps[idx],DRONE_REP_MULT));
    total_forces[idx] = add_points(&total_forces[idx],&scale_up_points(&max_danger_forces[idx],DANGER_MULT));

    /* ----------------------------------------------------------------------
     * Danger Mask already exist and probably not applicable in current setup
     * ----------------------------------------------------------------------
     * //danger based mask
     * if(cosine_sim(&DIRECTION_RAYS[idx],&total_forces[idx]) < 0.0) mask[idx]=false;
     */
    
    //more likely to perform alignment otherwise less likely to switch directions
    alignSim = cosine_sim(&DIRECTION_RAYS[idx],&alignment_force);
    constrainSim = fmaxf(SECTOR_COS_SIM,cosine_sim(&alignment_force,&vel));
    if(alignSim < constrainSim) total_forces[idx]=scale_up_points(&total_forces[idx],alignSim*((0.25-1.0)/(-1.0-constrainSim)));
  }

  //select strongest force as main force direction
  while((maxIdx < NUM_DIRECTION_RAYS) && !mask[maxIdx]) maxIdx+=1;
  for(int idx=maxIdx+1; idx<NUM_DIRECTION_RAYS; ++idx) 
    if(mask[idx] && (mag(&total_forces[idx]) > mag(&total_forces[maxIdx]))) 
      maxIdx = idx;
  if(maxIdx < NUM_DIRECTION_RAYS)
  {
    resulting_force = scale_up_points(&DIRECTION_RAYS[maxIdx],MAX_SPEED);

    //interpolate between main and strongest neighboring force
    magnitude = mag(&total_forces[maxIdx]);
    if(magnitude>1.0f) 
    {
      leftIdx = (maxIdx-1 + NUM_DIRECTION_RAYS)%NUM_DIRECTION_RAYS;
      rightIdx = (maxIdx+1 + NUM_DIRECTION_RAYS)%NUM_DIRECTION_RAYS;
      leftMag = mag(&total_forces[leftIdx]); 
      rightMag = mag(&total_forces[rightIdx]);
      neighborIdx = (leftMag<rightMag && mask[rightIdx])? rightIdx: (mask[leftIdx])? leftIdx: -1;
      if(neighborIdx>=0)
      {
        magnitude = mag(&total_forces[neighborIdx])/magnitude;
        resulting_force = add_points(&resulting_force,&scale_up_points(&DIRECTION_RAYS[neighborIdx],MAX_SPEED*magnitude));
      }
    }
    else if(magnitude<0.1f) scale_up_points(&vel,0.0f);
  } 
  else scale_up_points(&vel,0.0f);

  //set drone attributes accelleration, velocity, position
  acc = scale_up_points(&resulting_force,(MAX_SPEED/mag(&resulting_force)));

  struct EnuCoor_f* velocity = acInfoGetVelocityEnu_f(AC_ID);
  velocity->x = MAX_SPEED * tanhf(velocity->x+acc.x);
  velocity->y = MAX_SPEED * tanhf(velocity->y+acc.y);
  acInfoSetVelocityEnu_f(AC_ID,velocity);

  struct EnuCoor_i future_pos = *stateGetPositionEnu_i();
  future_pos.x += POS_BFP_OF_REAL(velocity->x)+POS_BFP_OF_REAL(FOLLOW_OFFSET_X);
  future_pos.y += POS_BFP_OF_REAL(velocity->y)+POS_BFP_OF_REAL(FOLLOW_OFFSET_Y);
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
