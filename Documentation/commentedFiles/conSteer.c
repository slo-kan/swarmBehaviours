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

/** @file conSteer.c
*  @brief Implementation of the context steering algorithm in paparazzi.
*  @details
*/


#include "swarm/swarm.h"
#include "swarm/swarm_info.h"
#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "subsystems/datalink/telemetry.h"
#include "errno.h"
#include "sys/stat.h"
#include "string.h"
#include "stdio.h"
#include "math.h"
#include "time.h"

#include "subsystems/navigation/waypoints.h"
#include "state.h"



/**
 * @brief follow offset in X direction(in ENU frame).
 */
#ifndef FOLLOW_OFFSET_X
#define FOLLOW_OFFSET_X 0.0f
#endif

/**
 * @brief follow offset in X direction(in ENU frame).
 */
#ifndef FOLLOW_OFFSET_Y
#define FOLLOW_OFFSET_Y 0.0f
#endif

/**
 * @brief defines the altitude of the aircraft above ground in meters.
 * @details since the script is only 2 dimensional the altitude of aircrafts is a constant.
 */
#ifndef FLIGHT_HEIGHT
#define FLIGHT_HEIGHT 47.0f
#endif

/**
 * @brief defines the maximum speed an aircraft can have in any direction.
 */
#ifndef MAX_SPEED
#define MAX_SPEED 3.0f //5.0f
#endif

/**
 * @brief defines the radius of Earth in meters.
 */
#ifndef GLOBE_RADIUS
#define GLOBE_RADIUS 6371000
#endif

/**
 * @brief defines the number of directional rays(entities) for context(map)
 */
#ifndef NUM_DIRECTION_RAYS
#define NUM_DIRECTION_RAYS 8
#endif

/**
* @brief is a  multiplier parameter for all the forces calculated.
*/
#ifndef GRAVITY
#define GRAVITY 1.985f
#endif

/**
 * @brief multiplier for goal attraction force in context maps.
 */
#ifndef GOAL_MULT
#define GOAL_MULT 5.0f  
#endif

/**
 * @brief is a multiplication parameter for attraction between aircrafts and goal points.
 */
#ifndef GOAL_MAX
#define GOAL_MAX 4.0f
#endif


/**
 * @brief multiplier for danger repulson force in context maps-
 */
#ifndef DANGER_MULT
#define DANGER_MULT 10.0f  
#endif

/**
 * @brief distance for masking for danger points in drone maps.
 */
// d_tc parameter
#ifndef DANGER_TO_CLOSE
#define DANGER_TO_CLOSE 8.0f
#endif

/**
 * @brief distance after which the repulsion force calculating function changes from linear to exponential.
 */
// d_co parameter
#ifndef DANGER_CUT_OFF
#define DANGER_CUT_OFF 5.0f
#endif

/**
 * @brief multiplication variable for the repulsion force between aircraft and danger points.
 */
// d_max parameter
#ifndef DANGER_MAX
#define DANGER_MAX 4.0f
#endif


/**
 * @brief multiplier for aircraft attraction force in context maps.
 */
// da_mult parameter
#ifndef DRONE_ATT_MULT
#define DRONE_ATT_MULT 0.6f 
#endif

/**
 * @brief minimum distance between aircrafts to calculate the attraction force.
 */
// da_co parameter
#ifndef DRONE_ATT_CUT_OFF
#define DRONE_ATT_CUT_OFF 20.0f  
#endif


/**
 * @brief maximum diatance upto which aircrafts attraction forces are calculated for drone maps.
 */
// da_sd parameter
#ifndef SWARM_DIST
#define SWARM_DIST 25.0f 
#endif

/**
 * @brief distance for drones for masking for danger maps.
 */
// dr_dtc parameter
#ifndef DRONE_TO_CLOSE
#define DRONE_TO_CLOSE 12.5f 
#endif

/**
 * @brief multiplier for aircraft repulsion force in context maps.                                             
 */
// dr_mult parameter
#ifndef DRONE_REP_MULT
#define DRONE_REP_MULT 6.0f 
#endif

/**
 * @brief multiplication variable for the repulsion force between aircrafts.
 */
// dr_max parameter
#ifndef DRONE_REP_MAX
#define DRONE_REP_MAX 10.0f
#endif


/**
 * @brief defines the multiplication parameter for alignment with other drones.
 */
#ifndef ALIGN_MULT
#define ALIGN_MULT 1.25f  //1/8 -> for goal mult = 8
#endif

/**
 * @brief defines the multiplication parameter for alignment with previous movement(depicts inertia).
 */
#ifndef SELF_ALIGN_MULT
#define SELF_ALIGN_MULT 0.5f  //1/12 -> for goal mult = 8
#endif

/**
 * @brief defines Swarm waypoint ids.
 */

#ifndef SWARM_WAYPOINT_ID
#error "Please define SWARM_WAYPOINT_ID with the ID of FOLLOW wp"
#endif

/**
 * @brief defines the first goal point id.
 */
#ifndef FIRST_GOAL_POINT_ID
#error "Please define the FIRST_GOAL_POINT_ID"
#endif

/**
 * @brief defines the last goal point id.
 */
#ifndef LAST_GOAL_POINT_ID
#error "Please define the LAST_GOAL_POINT_ID"
#endif

/**
 * @brief defines the first danger point id.
 */
#ifndef FIRST_DANGER_POINT_ID
#error "Please define the  FIRST_DANGER_POINT_ID"
#endif

/**
 * @brief defines the last danger point id.
 */
#ifndef LAST_DANGER_POINT_ID
#error "Please define the  LAST_DANGER_POINT_ID"
#endif

/**
 * @brief defines the first swarm member id.
 */
#ifndef FIRST_SWARM_MEMBER_ID
#error "Please define the FIRST_SWARM_MEMBER_ID"
#endif

/**
 * @brief defines the last swarm member id.
 */
#ifndef LAST_SWARM_MEMBER_ID
#error "Please define the LAST_SWARM_MEMBER_ID"
#endif


/** 
  @brief A class/structure Point consisting representing a vector with 2 dimensions, x and y.
  @class Point
 */
typedef struct Point {
  float x;
  float y;
} Point;

/**
 * @brief initializes a point.
 * @param x x coordinate of point as a float.
 * @param y y coordinate of point as a float.
 * @return Point
 **/
static Point init_point(float x, float y)
{
  Point new = {x,y};
  return new;
}

/**
 * @brief Function to calculate point from an angle.
 * @param radians angle in radians as a float.
 * @return Point
 **/
static Point point_from_angle(float radians)
{ return init_point(cosf(radians),sinf(radians)); }

/**
 * @brief subtracts a poin from another.
 * @param a a point as a Point.
 * @param b a point as a Point.
 * @return Point
 **/
static Point sub_points(Point* a, Point* b)
{ return init_point((a->x - b->x), (a->y - b->y)); }

/**
 * @brief adds a point to another.
 * @param a a point as a Point.
 * @param b a point as a Point.
 * @return Point
 **/
static Point add_points(Point* a, Point* b)
{ return init_point((a->x + b->x), (a->y + b->y)); }

/**
 * @brief scales up a point.
 * @param a a point as a Point.
 * @param s a variable to multiply as a float.
 * @return Point
 **/
static Point scale_up_points(Point* a, float s)
{ return init_point((a->x * s), (a->y * s)); }

/**
 * @brief scales down a point.
 * @param a a point as a Point.
 * @param s a variable to divide as float.
 * @return Point
 **/
static Point scale_down_points(Point* a, float s)
{ return init_point((a->x / s), (a->y / s)); }

/**
 * @brief dot product of two points.
 * @param a a point as a Point.
 * @param b a point as a Point.
 * @return dot product as float
 **/
static float dot(Point* a, Point* b)
{ return (a->x*b->x + a->y*b->y); }

/**
 * @brief magnitude of a point.
 * @param a a point as a Point.
 * @return magnitude as float
 **/
static float mag(Point* a)
{ return sqrtf(a->x*a->x + a->y*a->y ); }//+ a->z*a->z); }

/**
 * @brief cosine similarity between two points.
 * @param a a point as a Point.
 * @param b a point as a Point.
 * @return cosine similarity as float
 **/
static float cosine_sim(Point* a, Point* b)
{ return dot(a,b)/(mag(a)*mag(b)); }

/**
 * @brief distance between two points.
 * @param a a point as a Point.
 * @param b a point as a Point.
 * @return distance as float
 **/
static float dist_points(Point* a, Point* b)
{ 
  Point dist = sub_points(a,b); 
  return mag(&dist); 
}

/** @brief set point struct with given coordinates.
 * @details if point == null => do nothing.
 * @param p a point to set to as point.
 * @param x x coordinate of point as a float.
 * @param y y coordinate of point as a float.
 * 
 **/
static void set_points(Point* p, float x, float y)
{ 
  if(p!=NULL) 
  {
    p->x = x; 
    p->y = y;
  }
}

/**
 * @brief returns the point with higher magnitude.
 * @param a a point as a Point.
 * @param b a point as a Point.
 * @return Point 
 **/
static Point strongest_force(Point* a, Point* b)
{
  if(mag(a)>mag(b)) return *a;
  return *b; 
}



//simple data structures
/** @brief defines debug message
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
 **/
 
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

/**
 * @brief message that contains "reached status" of each copter
 * @param wp_id id of approched waypoint
 * @param own_pos position of copter as point
 * @param reached bool if goal is reached or not
 **/
struct Message_Goal {
    uint8_t wp_id;                 ///< @brief id of approched waypoint
    struct LlaCoor_f own_pos;      ///< @brief position of copter
    bool reached;                  ///< @brief bool if goal is reached or not
};

/**
 * @brief message that contains a dummy variable
 * @param z dummy varible as float
 **/
struct DummyZ {
   float z;
};



//global variables
/** @brief timespec to store time */
static struct timespec tp = {0, 0};
/** @brief cosine similarity of a sector */
static float SECTOR_COS_SIM = cosf(M_PI/NUM_DIRECTION_RAYS);
/** @brief array of points(direction rays) */
static Point DIRECTION_RAYS[NUM_DIRECTION_RAYS] = {{0.0f,0.0f}};
/** @brief acceleration */
static Point acc = {0.0f, 0.0f};
/** @brief velocity */
static Point vel = {0.0f, 0.0f};
/** @brief position */
static Point pos = {0.0f, 0.0f};
/** @brief logfile */
static FILE* LOG_FILE = NULL;
/** @brief debug file */
static FILE* DEBUG_FILE = NULL;
/** @brief swarm info debug file */
static FILE* SWARM_INFO_DEBUG_FILE = NULL;
/** @brief file name buffer */
static char file_name[150]={'\0'};
/** @brief tick count */
static int tick = 0;

/** @brief debug message */
static struct Message_Debug msg = {{0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f},0,{0.0f,0.0f,0.0f},0.0f,0.0f,false,{0.0f,0.0f,0.0f},0.0f,0.0f,false};
/** @brief goal message */
static struct Message_Goal syncLink = {0,{0.0f,0.0f,0.0f},false};
/** @brief attraction pointe */
static struct LlaCoor_f att_point = {0.0f,0.0f,0.0f};
/** @brief drone position */
static struct LlaCoor_f current_pos = {0.0f,0.0f,0.0f};
/** @brief dummy variable */
static struct DummyZ dummy = {0.0f};

//LlaCoor_f = {lat,lon,alt} specified in radients in a floating point number format(as specified in the paparazzi documentation)
//LlaCoor_i = {lat,lon,alt} specified in degrees in an integer format(as specified in the paparazzi documentation)
//EnuCoor_i = {x,y,z} specified in meters in an integer format(as specified in the paparazzi documentation)
//EnuCoor_f = {x,y,z} specified in meters in an floating point number format(as specified in the paparazzi documentation)



/**
 * @brief writes debug log to debug file.
 * @param interest_forces interest forces of aircrafts.
 * @param danger_forces danger forces of aircrafts.
 * @param member_att_forces attraction forces between aircrafts.
 * @param member_rep_forces repulsion forces between aircrafts.
 * @param score score of aircrafts.
 * @param alignmentMap allignment of aircraft with other aircrafts.
 * @param selfAlignmentMap self allignment map of aircraft.
 * @param mask mask map of aircraft.
 * @param alignment_force allignment force of aircraft.
 * @param choosen_dir chosen direction of aircraft.
 * @param neighbour_dir neighbor direction af chosen direction.
 */
static void write_to_debug_file(Point* interest_forces, Point* danger_forces, Point* member_att_forces, Point* member_rep_forces, //Point* total_forces
 float* score, float* alignmentMap, float* selfAlignmentMap, bool* mask, Point alignment_force, int choosen_dir, int neighbour_dir)
{
  if(DEBUG_FILE != NULL)
  {
    fprintf(DEBUG_FILE,"%11.6f, %10.4f, %10.4f, %10.4f, %10.4f, %10.4f, %10.4f, %3d, %3d", 
            get_sys_time_float(), pos.x, pos.y, vel.x, vel.y, acc.x, acc.y, choosen_dir, neighbour_dir);
    fprintf(DEBUG_FILE,", %8.4f, %8.4f", alignment_force.x, alignment_force.y );
    for (int it=0; it<NUM_DIRECTION_RAYS; ++it) fprintf(DEBUG_FILE,", %1d", mask[it]);
    for (int it=0; it<NUM_DIRECTION_RAYS; ++it) fprintf(DEBUG_FILE,", %8.4f", score[it]);
    for (int it=0; it<NUM_DIRECTION_RAYS; ++it) fprintf(DEBUG_FILE,", %8.4f", alignmentMap[it]);
    for (int it=0; it<NUM_DIRECTION_RAYS; ++it) fprintf(DEBUG_FILE,", %8.4f", selfAlignmentMap[it]);
    for (int it=0; it<NUM_DIRECTION_RAYS; ++it) fprintf(DEBUG_FILE,", %10.4f, %10.4f", interest_forces[it].x, interest_forces[it].y);
    for (int it=0; it<NUM_DIRECTION_RAYS; ++it) fprintf(DEBUG_FILE,", %10.4f, %10.4f", member_att_forces[it].x, member_att_forces[it].y);
    for (int it=0; it<NUM_DIRECTION_RAYS; ++it) fprintf(DEBUG_FILE,", %10.4f, %10.4f", member_rep_forces[it].x, member_rep_forces[it].y);
    for (int it=0; it<NUM_DIRECTION_RAYS; ++it) fprintf(DEBUG_FILE,", %10.4f, %10.4f", danger_forces[it].x, danger_forces[it].y);
    fprintf(DEBUG_FILE,"\n");
  }
}

/**
 * @brief Creates a debug file.
 * @details 
 * @return -1 when error occures, else 0
 */
static int create_debug_file()
{
  sprintf(file_name,"/home/finkensim/paparazzi/logs/ConSteer/ac_%d_debug.csv",AC_ID);
  if(mkdir("/home/finkensim/paparazzi/", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)<0 && errno!=EEXIST) return -1;
  if(mkdir("/home/finkensim/paparazzi/logs", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)<0 && errno!=EEXIST) return -1;
  if(mkdir("/home/finkensim/paparazzi/logs/ConSteer", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)<0 && errno!=EEXIST) return -1;
  DEBUG_FILE = fopen(file_name,"w");
  if(DEBUG_FILE == NULL) printf("\nFile could not be opened!");
  else{
    fprintf(DEBUG_FILE,"Startup-Time: %11.6f\n",get_sys_time_float());
    fprintf(DEBUG_FILE,"timestamp, pos_x, pos_y, vel_x, vel_y, acc_x, acc_y, choosen_dir, neighbour_dir, alignment_force_x, alignment_force_y");
    for (int it=0; it<NUM_DIRECTION_RAYS; ++it) fprintf(DEBUG_FILE,", %d_mask",it);
    for (int it=0; it<NUM_DIRECTION_RAYS; ++it) fprintf(DEBUG_FILE,", %d_score",it);
    for (int it=0; it<NUM_DIRECTION_RAYS; ++it) fprintf(DEBUG_FILE,", %d_align",it);
    for (int it=0; it<NUM_DIRECTION_RAYS; ++it) fprintf(DEBUG_FILE,", %d_self_align",it);
    for (int it=0; it<NUM_DIRECTION_RAYS; ++it) fprintf(DEBUG_FILE,", %d_max_interest_force_x, %d_max_interest_force_y",it,it);
    for (int it=0; it<NUM_DIRECTION_RAYS; ++it) fprintf(DEBUG_FILE,", %d_max_member_att_force_x, %d_max_member_att_force_y",it,it);
    for (int it=0; it<NUM_DIRECTION_RAYS; ++it) fprintf(DEBUG_FILE,", %d_max_member_rep_force_x, %d_max_member_rep_force_y",it,it);
    for (int it=0; it<NUM_DIRECTION_RAYS; ++it) fprintf(DEBUG_FILE,", %d_max_danger_force_x, %d_max_danger_force_y",it,it);
    fprintf(DEBUG_FILE,"\n");
  }
  return 0;
}

/**
 * @brief writes log to log file.
 */
static void write_to_log_file()
{
  if(LOG_FILE != NULL)
  {
    fprintf(LOG_FILE,"[%11.6f] pos: %12.8f, %12.8f / %10.4f, %10.4f; vel: %10.4f, %10.4f; acc: %10.4f, %10.4f\n",
            get_sys_time_float(), current_pos.lat*(180/M_PI), current_pos.lon*(180/M_PI), pos.x, pos.y, vel.x, vel.y, acc.x, acc.y);
  }
}

/**
 * @brief Create a log file object.
 * @return -1 when error occures, else 0.
 */
static int create_log_file()
{
  sprintf(file_name,"/home/finkensim/paparazzi/logs/ConSteer/ac_%d.log",AC_ID);
  if(mkdir("/home/finkensim/paparazzi/", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)<0 && errno!=EEXIST) return -1;
  if(mkdir("/home/finkensim/paparazzi/logs", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)<0 && errno!=EEXIST) return -1;
  if(mkdir("/home/finkensim/paparazzi/logs/ConSteer", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)<0 && errno!=EEXIST) return -1;
  LOG_FILE = fopen(file_name,"w");
  if(LOG_FILE == NULL) printf("\nFile could not be opened!");
  else fprintf(LOG_FILE,"Startup-Time: %11.6f\n",get_sys_time_float());
  return 0;
}

/**
 * @brief writes swarm log to swarm info log file.
 * @param others array of positions of aircrafts
 * @param size number of aircrafts as int
 */
static void write_to_swarm_info_debug_file(Point* others, int size)
{
  if(SWARM_INFO_DEBUG_FILE != NULL)
  {
    clock_gettime(CLOCK_MONOTONIC,&tp);
    fprintf(SWARM_INFO_DEBUG_FILE,"%11.6f, %li.%09li, %10.4f, %10.4f",
            get_sys_time_float(), tp.tv_sec, tp.tv_nsec, pos.x, pos.y);
    for (int it=0; it<size; ++it) fprintf(SWARM_INFO_DEBUG_FILE,", %10.4f, %10.4f", others[it].x, others[it].y);
    fprintf(SWARM_INFO_DEBUG_FILE,"\n");
  }
}

/**
 * @brief Creates a debug file for swarm info.
 * @details 
 * @param start first aircraft id
 * @param end last aircraft id
 * @return -1 when error occures, else 0
 */
static int create_swarm_info_debug_file(int start, int end)
{
  sprintf(file_name,"/home/finkensim/paparazzi/logs/ConSteer/ac_%d_swarm_info.csv",AC_ID);
  if(mkdir("/home/finkensim/paparazzi/", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)<0 && errno!=EEXIST) return -1;
  if(mkdir("/home/finkensim/paparazzi/logs", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)<0 && errno!=EEXIST) return -1;
  if(mkdir("/home/finkensim/paparazzi/logs/ConSteer", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)<0 && errno!=EEXIST) return -1;
  SWARM_INFO_DEBUG_FILE = fopen(file_name,"w");
  if(SWARM_INFO_DEBUG_FILE == NULL) printf("\nFile could not be opened!");
  else
  { 
    clock_gettime(CLOCK_MONOTONIC,&tp);
    fprintf(SWARM_INFO_DEBUG_FILE,"Startup-Time: %11.6f at %li.%09li sec\n",get_sys_time_float(), tp.tv_sec, tp.tv_nsec);
    fprintf(SWARM_INFO_DEBUG_FILE,"drone_timestamp, sys_timestamp, own_x, own_y");
    for (int it=start; it<=end; ++it) fprintf(SWARM_INFO_DEBUG_FILE,", member_%d_x, member_%d_y", it, it);
    fprintf(SWARM_INFO_DEBUG_FILE,"\n");
  }
  return 0;
}

/**
 * @brief send acceleration info
 * @param trans transport channel
 * @param dev link device
 */
static void send_acc_info(struct transport_tx *trans, struct link_device *dev) 
{	pprz_msg_send_ACC(trans, dev, AC_ID, &acc.x, &acc.y, &dummy.z); }

/**
 * @brief send goal info
 * @param trans transport channel
 * @param dev link device
 */
static void send_goal_info(struct transport_tx *trans, struct link_device *dev) 
{	pprz_msg_send_GOAL_ACHIEVED(trans, dev, AC_ID, &syncLink.wp_id, &syncLink.own_pos.lat, &syncLink.own_pos.lon, &syncLink.own_pos.alt, (uint8_t*)&syncLink.reached); }

/**
 * @brief send attraction repulsion info
 * @param trans transport channel
 * @param dev link device
 */
static void send_attract_and_repulse_info(struct transport_tx *trans, struct link_device *dev) 
{ pprz_msg_send_ATTREP(trans, dev, AC_ID, &msg.own_pos.x, &msg.own_pos.y, &msg.own_pos.z, &msg.target_pos.x, &msg.target_pos.y, &msg.target_pos.z, &msg.target_ac_id, &msg.attraction_force.x, &msg.attraction_force.y, &msg.attraction_force.z, &msg.attraction_d, &msg.attraction_strength, (uint8_t*)&msg.attraction, &msg.repulsion_force.x, &msg.repulsion_force.y, &msg.repulsion_force.z, &msg.repulsion_d, &msg.repulsion_strength, (uint8_t*)&msg.repulsion); }

/** 
 * @brief initialize swarm
 * @details  calculate the direction rays (end)points, register goal, acceleration, and attraction repulsion info sending and create log and debug files
 */
void swarm_init(void) 
{
  int start = FIRST_SWARM_MEMBER_ID;
  int end = LAST_SWARM_MEMBER_ID;
  for(int it=0; it<NUM_DIRECTION_RAYS; ++it)
  {
    float angle = it * (2*M_PI)/NUM_DIRECTION_RAYS; //M_PI
    DIRECTION_RAYS[it] = point_from_angle(angle);                                                                                                                                                                                                                                                         ;
  }
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GOAL_ACHIEVED, send_goal_info);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ACC, send_acc_info);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ATTREP, send_attract_and_repulse_info);
  if(create_log_file()<0) printf("\nERROR: Could not open log file due to directory access/creation issues!");
  if(create_debug_file()<0) printf("\nERROR: Could not open debug file due to directory access/creation issues!");
  if(create_swarm_info_debug_file(start,end)<0) printf("\nERROR: Could not open debug file due to directory access/creation issues!");
}

/**
 * @brief converts from LlaCoor_i to LlaCoor_f by reducing to float value and switching from degrees to radians
 * @param point LlaCoor_i point that is to be converted to LlaCoor_f
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
 * @brief gets position of aircraft in local ENU coordinates(float).
 * @param[in] ac_id aircraft id of aircraft info to get
 */
static struct EnuCoor_f *getPositionEnu_f(uint8_t ac_id)
{ return (ti_acs[ti_acs_id[ac_id]].ac_id != ac_id)? NULL: acInfoGetPositionEnu_f(ac_id); }


/**
 * @brief gets the metric distance between two points given in the LlaCoor_f format
 * @param own_pos own position as point
 * @param goal_pos goal position as point
 */
static float getDistance(struct LlaCoor_f* own_pos, struct LlaCoor_f* goal_pos)
{
  return GLOBE_RADIUS * acos(
    sin(own_pos->lat)*sin(goal_pos->lat) + 
    cos(own_pos->lat)*cos(goal_pos->lat) * 
    cos(own_pos->lon - goal_pos->lon));
}

/**
 * @brief updates the content of aircraft via the periodically sent goal_achieved messages
 * @param own_pos own position as point
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
 * @brief calculates repulsion forces to other aircrafts.
 * @details 
 * @param pos position of own aircraft as Point.
 * @param target position of target aircraft as Point.
 * @param max_val multiplication variable for the repulsion force as float.
 * @return repulsion force between the aircraft and target as Point.
 */
static Point linear_Repulsion(Point* pos, Point* target, float max_val)
{
  Point force = sub_points(target, pos);
  float magnitude = mag(&force);
  float d = magnitude; //fminf(magnitude, limit);
  float strength = (-1)*max_val*GRAVITY/fmaxf(1.0f,d);
  force = scale_up_points(&force,(strength/magnitude));
  return force;
}

/**
 * @brief calculates attraction forces to other aircrafts.
 * @details 
 * @param pos position of own aircraft as Point.
 * @param target position of target aircraft as Point.
 * @param cutoff minimum distance between aircrafts to calculate the attraction force as float.
 * @return attraction force between the aircraft and target as Point.
 */
static Point log_Attraction(Point* pos, Point* target, float cutOff)
{
  Point force = sub_points(target, pos);
  float magnitude = mag(&force);
  float d = magnitude; //fminf(magnitude, limit);
  float strength = 0;
  if(d>cutOff) strength = log(1+(d-cutOff))*GRAVITY; 
  force = scale_up_points(&force,(strength/magnitude));
  return force;
}

/**
 * @brief calculates attraction forces to goal points.
 * @details 
 * @param pos position of own aircraft as Point.
 * @param target position of goal point as Point.
 * @param max_val multiplication variable for the attraction force as float.
 * @return attraction force between the aircraft and goal point as Point.
 */
static Point linear_Attraction(Point* pos, Point* target, float max_val)
{
  Point force = sub_points(target, pos);
  float magnitude = mag(&force);
  float d = magnitude; //fminf(magnitude, limit);
  float strength = (GRAVITY * max_val)/fmaxf(d,1.0f);
  force = scale_up_points(&force,(strength/magnitude));
  return force;
}

/**
 * @brief calculates repulsion forces to danger points.
 * @details 
 * @param pos position of own aircraft as Point.
 * @param target position of danger point as Point.
 * @param cutoff distance after which the repulsion function changes from linear to exponential, as float.
 * @param max_val multiplication variable for the repulsion force as float.
 * @return repulsion force between the aircraft and danger point as Point.
 */
static Point limExp_Repulsion(Point* pos, Point* target, float cutOff, float max_val)
{
  Point force = sub_points(target, pos);
  float magnitude = mag(&force);
  float strength;
  float d = magnitude; //fminf(magnitude, limit);
  if (d<cutOff) strength = (-1)*(GRAVITY * max_val)/fmaxf(d,1.0f); //linear_rep_limit_close
  else strength = (-1)*expf((-1)* GRAVITY * logf(d));     //exp_rep_limit_mid_far
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
  int AC_AMOUNT = (LAST_SWARM_MEMBER_ID-FIRST_SWARM_MEMBER_ID+1);
  set_points(&pos, stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y); 
  set_points(&vel, stateGetSpeedEnu_f()->x, stateGetSpeedEnu_f()->y);
  Point danger, goal, other_pos, other_vel, dist, att_force, rep_force;
  Point alignment_force = {0.0f, 0.0f};
  Point scaled_force,resulting_force = {0.0f,0.0f};
  Point max_interest_forces[NUM_DIRECTION_RAYS] = {{0.0f,0.0f}};
  Point max_danger_forces[NUM_DIRECTION_RAYS] = {{0.0f,0.0f}};
  Point max_member_atts[NUM_DIRECTION_RAYS] = {{0.0f,0.0f}};
  Point max_member_reps[NUM_DIRECTION_RAYS] = {{0.0f,0.0f}};
  Point other_acs[AC_AMOUNT];
  bool mask[NUM_DIRECTION_RAYS] = {true};
  float score[NUM_DIRECTION_RAYS] = {0.0f};
  float alignmentMap[NUM_DIRECTION_RAYS] = {0.0f};
  float selfAlignmentMap[NUM_DIRECTION_RAYS] = {0.0f};
  float member_dist, alignSim, velSim, magnitude;
  float alignment_counter = 0.0f;
  int leftIdx,rightIdx,neighborIdx;
  int maxIdx = 0;
  
  for(int idx=0; idx<NUM_DIRECTION_RAYS; ++idx) 
  {
    mask[idx] = true; 
    score[idx] = 0.0f;
    alignmentMap[idx] = 0.0f;
    selfAlignmentMap[idx] = 0.0f;
    set_points((max_interest_forces+idx), 0.0f,0.0f);
    set_points((max_danger_forces+idx), 0.0f,0.0f);
    set_points((max_member_atts+idx), 0.0f,0.0f);
    set_points((max_member_reps+idx), 0.0f,0.0f);
  }
  for(int idx=0; idx<(LAST_SWARM_MEMBER_ID-FIRST_SWARM_MEMBER_ID+1); ++idx)
    set_points((other_acs+idx), 0.0f,0.0f);
  
  /*
   +++++++++++++++++++++++++++
   + Context Map Preparation +
   +++++++++++++++++++++++++++
  
   create all direction segments for each map based on maximum magnitude of force 
   and create a mask based on distance
  */
  for(int idx=0; idx<NUM_DIRECTION_RAYS; ++idx)
  {
    /* for each aircraft, distance to the goal point is calulated and if the cosine similarity is greater than 
    sector cosine similarity, the attraction force is calculated and maximum is assigned as max interest force*/
    for(uint8_t wp_id = FIRST_GOAL_POINT_ID; wp_id < LAST_GOAL_POINT_ID; ++wp_id)
    {
      set_points(&goal,waypoint_get_x(wp_id),waypoint_get_y(wp_id));
      dist = sub_points(&goal,&pos);
      if(cosine_sim((DIRECTION_RAYS+idx), &dist) >= SECTOR_COS_SIM) 
      {
        att_force = linear_Attraction(&pos,&goal,GOAL_MAX);
        max_interest_forces[idx] = strongest_force((max_interest_forces+idx),&att_force);
      }
    }

    //DRONE MAPS & MASK
    /* for each aircraft, distance to the every other is calulated and the if the distance is smaller than drone_to_close then
     it is maked or if the distance is less than swarm_dist, the alignment forces, and attraction and repulsion forces are calculated
     amd the maximum of those are assigned. */
    for(uint8_t ac_id = FIRST_SWARM_MEMBER_ID; ac_id <= LAST_SWARM_MEMBER_ID; ++ac_id)
    {
      int index = ac_id-FIRST_SWARM_MEMBER_ID;
      if(ac_id != AC_ID && getPositionEnu_f(ac_id) != NULL) 
      {
        set_points(&other_pos,getPositionEnu_f(ac_id)->x,getPositionEnu_f(ac_id)->y);
        set_points((other_acs+index),other_pos.x,other_pos.y);
        dist = sub_points(&other_pos,&pos);
        //rotated = rotate2D(&dist,M_PI); 
        if(cosine_sim((DIRECTION_RAYS+idx), &dist) >= SECTOR_COS_SIM) 
        {
          member_dist = dist_points(&other_pos,&pos);
          if(member_dist<=DRONE_TO_CLOSE) mask[idx] = false;
          else if (member_dist<=SWARM_DIST) 
          {
            set_points(&other_vel,acInfoGetVelocityEnu_f(ac_id)->x,acInfoGetVelocityEnu_f(ac_id)->y);
            alignment_force = add_points(&alignment_force,&other_vel);
            alignment_counter+=1.0f;
          }
          att_force = log_Attraction(&pos,&other_pos,DRONE_ATT_CUT_OFF);
          max_member_atts[idx] = strongest_force((max_member_atts+idx),&att_force);
          
          rep_force = linear_Repulsion(&pos,&other_pos,DRONE_REP_MAX);
          max_member_reps[idx] = strongest_force((max_member_reps+idx), &rep_force); 
        }
      }
      else set_points((other_acs+index),stateGetPositionEnu_f()->x,stateGetPositionEnu_f()->y);
    }

    //DANGER MAP & MASK
    /* for each aircraft, distance to the danger point is calulated and if the distance is smaller than drone_to_close then
    the repulsion force is calculated and maximum is assigned as max danger force. */
    for(uint8_t wp_id = FIRST_DANGER_POINT_ID; wp_id < LAST_DANGER_POINT_ID; ++wp_id)
    {
      set_points (&danger,waypoint_get_x(wp_id),waypoint_get_y(wp_id));
      dist = sub_points(&danger,&pos);
      //rotated = rotate2D(&dist,M_PI);
      if(cosine_sim((DIRECTION_RAYS+idx), &dist) >= SECTOR_COS_SIM) 
      {
        if(dist_points(&danger,&pos)<=DANGER_TO_CLOSE) mask[idx] = false;
        
        rep_force = limExp_Repulsion(&pos,&danger,DANGER_CUT_OFF,DANGER_MAX);
        max_danger_forces[idx] = strongest_force((max_danger_forces+idx),&rep_force);
      }
    }
  }


  /*
   ++++++++++++++++++++++++++
   + Context Map Evaluation +
   ++++++++++++++++++++++++++
  
   evaluate context steering behavior
  */

  /* calculate scores with respect to context map weightings and swarm alignment */
  if(alignment_counter>0.0f) alignment_force = scale_down_points(&alignment_force,alignment_counter);
  for(int idx=0; idx<NUM_DIRECTION_RAYS; ++idx)
  {
    score[idx] = 0.000f;
    score[idx] = mag((max_interest_forces+idx))*GOAL_MULT;
    score[idx] = score[idx]+mag((max_member_atts+idx))*DRONE_ATT_MULT;
    score[idx] = score[idx]-mag((max_member_reps+idx))*DRONE_REP_MULT;
    score[idx] = score[idx]-mag((max_danger_forces+idx))*DANGER_MULT;

    /* more likely to perform alignment otherwise less likely to switch directions */
    alignSim = cosine_sim((DIRECTION_RAYS+idx),&alignment_force);
    velSim = cosine_sim((DIRECTION_RAYS+idx),&vel);
    alignmentMap[idx] = alignSim * (mag(&alignment_force)/MAX_SPEED) * ALIGN_MULT;
    selfAlignmentMap[idx] = velSim * (mag(&vel)/MAX_SPEED) * SELF_ALIGN_MULT;
    if(mag(&alignment_force)>0.01f) score[idx] = score[idx] + alignmentMap[idx];
    if(mag(&vel)>0.01f) score[idx] = score[idx] + selfAlignmentMap[idx];
  }


  /*select strongest force as main force direction */
  while((maxIdx < NUM_DIRECTION_RAYS) && !mask[maxIdx]) maxIdx+=1;
  for(int idx=maxIdx+1; idx<NUM_DIRECTION_RAYS; ++idx) 
    if(mask[idx] && (score[idx] > score[maxIdx])) maxIdx = idx;

  if(maxIdx < NUM_DIRECTION_RAYS)
  {
    if(fabsf(score[maxIdx])>0.002f)
    {
      /*interpolate between main and strongest neighboring force */
      leftIdx = (maxIdx-1 + NUM_DIRECTION_RAYS)%NUM_DIRECTION_RAYS;
      rightIdx = (maxIdx+1 + NUM_DIRECTION_RAYS)%NUM_DIRECTION_RAYS;
      neighborIdx = -1;
      if(score[leftIdx]<score[rightIdx])
      {
        if(mask[rightIdx]) neighborIdx = rightIdx;
        else if(mask[leftIdx]) neighborIdx = leftIdx;
      }
      else
      {
        if(mask[leftIdx]) neighborIdx = leftIdx;
        else if(mask[rightIdx]) neighborIdx = rightIdx;
      }
      if(neighborIdx>=0)
      {
        magnitude = score[maxIdx]/(score[maxIdx]+score[neighborIdx]);
        resulting_force = scale_up_points((DIRECTION_RAYS+maxIdx),magnitude);
        scaled_force = scale_up_points((DIRECTION_RAYS+neighborIdx),(1.0f-magnitude));
        resulting_force = add_points(&resulting_force,&scaled_force);
      }
    }
    else resulting_force = scale_up_points((DIRECTION_RAYS+maxIdx),1.0f);
  } 
  else scale_up_points(&vel,0.0f);

  if(fabsf(score[maxIdx])<0.0001f)
  {
    scale_up_points(&vel,0.0f);
    scale_up_points(&acc,0.0f);
  }  
  else
  {
    /*set drone attributes accelleration, velocity, position */
    acc = scale_up_points(&resulting_force,(MAX_SPEED/mag(&resulting_force)));
  }

  struct EnuCoor_f* velocity = stateGetSpeedEnu_f();
  velocity->x = MAX_SPEED * tanhf(vel.x/MAX_SPEED+acc.x/MAX_SPEED);
  velocity->y = MAX_SPEED * tanhf(vel.y/MAX_SPEED+acc.y/MAX_SPEED);
  acInfoSetVelocityEnu_f(AC_ID,velocity);

  struct EnuCoor_i future_pos = *stateGetPositionEnu_i();
  future_pos.x += POS_BFP_OF_REAL(velocity->x);
  future_pos.y += POS_BFP_OF_REAL(velocity->y);
  future_pos.z = POS_BFP_OF_REAL(FLIGHT_HEIGHT);

  /* Move the waypoints */
  current_pos = *stateGetPositionLla_f();
  waypoint_set_enu_i(SWARM_WAYPOINT_ID, &future_pos);
  for(uint8_t wp_id = FIRST_GOAL_POINT_ID; wp_id < LAST_GOAL_POINT_ID; ++wp_id)
  {
    struct LlaCoor_i* way_point = waypoint_get_lla(wp_id);
    att_point = toFloatPointFormat(way_point);
    updateSyncLinkMsg(&current_pos, wp_id);
  }
  
  if(tick==0) 
  {
    tick = 3;
    write_to_log_file();
    write_to_debug_file(max_interest_forces, max_danger_forces, max_member_atts, max_member_reps, score, 
                        alignmentMap, selfAlignmentMap, mask, alignment_force, maxIdx, neighborIdx);
    write_to_swarm_info_debug_file(other_acs,AC_AMOUNT);
  }
  else --tick;
}
