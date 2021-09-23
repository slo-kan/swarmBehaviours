#!/usr/bin/python

#module imports
import sys
from os import path

SCENARIO_DIR = path.dirname(path.abspath(__file__))
FLIGHT_PLAN_DIR = path.normpath(path.join(SCENARIO_DIR, '../../../../conf/flight_plans/ovgu_swarm'))
COPTER_DIR = path.normpath(path.join(SCENARIO_DIR, '../../../../conf/airframes/ovgu'))

FLIGHT_PLAN_HEADER = '<!DOCTYPE flight_plan SYSTEM "../flight_plan.dtd">\n'+ \
'<flight_plan alt="47" ground_alt="44" '
FLIGHT_PLAN_BODY = ' max_dist_from_home="300" name="Rotorcraft Basic (Enac)" security_height="10">\n'+ \
  '<header>\n'+ \
    '#include "autopilot.h"\n'+ \
  '</header>\n'+ \
  '<waypoints>\n'+ \
    '<waypoint name="FOLLOW" x="0.0" y="0.0"/>\n'
FLIGHT_PLAN_END = '<waypoint name="HOME" x="0.0" y="0.0" alt="47"/>\n'+ \
  '</waypoints>\n'+ \
  '<blocks>\n'+ \
    '<block name="Wait GPS">\n'+ \
      '<call_once fun="NavKillThrottle()"/>\n'+ \
      '<while cond="!GpsFixValid()"/>\n'+ \
    '</block>\n'+ \
    '<block name="Geo init">\n'+ \
      '<while cond="LessThan(NavBlockTime(), 1)"/>\n'+ \
      '<call_once fun="NavSetGroundReferenceHere()"/>\n'+ \
      '<call_once fun="NavSetAltitudeReferenceHere()"/>\n'+ \
    '</block>\n'+ \
    '<block name="Holding point">\n'+ \
      '<call_once fun="NavKillThrottle()"/>\n'+ \
      '<attitude pitch="0" roll="0" throttle="0" vmode="throttle" until="NavBlockTime() > 20"/>\n'+ \
    '</block>\n'+ \
    '<block name="Start Engine">\n'+ \
      '<call_once fun="NavResurrect()"/>\n'+ \
      '<attitude pitch="0" roll="0" throttle="0" vmode="throttle" until="NavBlockTime() > 5"/>\n'+ \
    '</block>\n'+ \
    '<block name="Takeoff" strip_button="Takeoff" strip_icon="takeoff.png">\n'+ \
      '<exception cond="stateGetPositionEnu_f()->z > 1.0" deroute="START"/>\n'+ \
      '<call_once fun="NavSetWaypointHere(WP_HOME)"/>\n'+ \
      '<stay vmode="climb" climb="nav_climb_vspeed" wp="HOME"/>\n'+ \
    '</block>\n'+ \
    '<block name="Standby" strip_button="Standby" strip_icon="home.png">\n'+ \
      '<stay wp="HOME"/>\n'+ \
    '</block>\n'+ \
    '<block name="START">\n'+ \
      '<go wp="HOME" approaching_time="0"/>\n'+ \
      '<deroute block="FOLLOWING"/>\n'+ \
    '</block>\n'+ \
    '<block name="FOLLOWING">\n'+ \
      '<go wp="FOLLOW" approaching_time="0"/>\n'+ \
      '<deroute block="FOLLOWING"/>\n'+ \
    '</block>\n'+ \
    '<block name="flare">\n'+ \
      '<exception cond="NavDetectGround()" deroute="Holding point"/>\n'+ \
      '<exception cond="!nav_is_in_flight()" deroute="landed"/>\n'+ \
      '<call_once fun="NavStartDetectGround()"/>\n'+ \
      '<stay climb="nav_descend_vspeed" vmode="climb" wp="HOME"/>\n'+ \
    '</block>\n'+ \
    '<block name="landed">\n'+ \
      '<attitude pitch="0" roll="0" throttle="0" vmode="throttle" until="FALSE"/>\n'+ \
    '</block>\n'+ \
  '</blocks>\n'+ \
'</flight_plan>'




COPTER_HEADER = '<!DOCTYPE airframe SYSTEM "../airframe.dtd">\n'+ \
'\n'+ \
'<!-- this is a quadrotor frame equiped with\n'+ \
    ' * Autopilot:   Lisa/M 2.0             http://wiki.paparazziuav.org/wiki/Lisa/M_v20 \n'+ \
    ' * IMU:         Aspirin 2.2            http://wiki.paparazziuav.org/wiki/AspirinIMU \n'+ \
    ' * Actuators:   PWM motor controllers  http://wiki.paparazziuav.org/wiki/Subsystem/actuators#PWM_Supervision \n'+ \
    ' * GPS:         Ublox                  http://wiki.paparazziuav.org/wiki/Subsystem/gps \n'+ \
    ' * RC:          two Spektrum sats      http://wiki.paparazziuav.org/wiki/Subsystem/radio_control#Spektrum \n'+ \
'-->\n'+ \
'\n'+ \
'<airframe name="Quadrotor LisaM_2.0 pwm">\n'+ \
  '\n'+ \
  '<firmware name="rotorcraft">\n'+ \
    '<target name="ap" board="lisa_m_2.0">\n'+ \
      '<!-- MPU6000 is configured to output data at 2kHz, but polled at 512Hz PERIODIC_FREQUENCY -->\n'+ \
    '</target>\n'+ \
    '\n'+ \
    '<target name="nps" board="pc">\n'+ \
	    '<module name="fdm" type="jsbsim"/>\n'+ \
    '</target>\n'+ \
    '\n'+ \
    '<module name="radio_control" type="spektrum">\n'+ \
      '<define name="RADIO_MODE" value="RADIO_AUX1"/>\n'+ \
      '<configure name="USE_SECONDARY_SPEKTRUM_RECEIVER" value="1"/>\n'+ \
    '</module>\n'+ \
    '<module name="motor_mixing"/>\n'+ \
    '<module name="actuators"     type="pwm">\n'+ \
      '<define name="SERVO_HZ" value="400"/>\n'+ \
      '<!--define name="USE_SERVOS_7AND8"/-->\n'+ \
    '</module>\n'+ \
    '\n'+ \
    '<module name="telemetry"     type="transparent"/>\n'+ \
    '<module name="imu"           type="aspirin_v2.2"/>\n'+ \
    '<module name="gps"           type="ublox"/>\n'+ \
    '<module name="stabilization" type="int_quat"/>\n'+ \
    '<module name="ahrs"          type="int_cmpl_quat">\n'+ \
      '<define name="AHRS_GRAVITY_HEURISTIC_FACTOR" value="30"/>\n'+ \
    '</module>\n'+ \
    '<module name="ins"/>\n'+ \
    '<module name="swarm">\n'
'''    
  <define name="REPELL_POINT_ID" value="3"/> 
  <define name="ATTRACTION_POINT_ID" value="2"/>
'''
COPTER_END = '<define name="SWARM_WAYPOINT_ID" value="1"/>\n'+ \
      '<define name="FIRST_SWARM_MEMBER_ID" value="30"/>\n'+ \
      '<define name="LAST_SWARM_MEMBER_ID" value="39"/>\n'+ \
    '</module>\n'+ \
    '<!--module name="traffic_info"/-->\n'+ \
    '\n'+ \
    '<module name="gps" type="ubx_ucenter"/>\n'+ \
    '<module name="geo_mag"/>\n'+ \
    '<module name="air_data"/>\n'+ \
    '\n'+ \
    '<!--define name="KILL_ON_GROUND_DETECT" value="TRUE"/-->\n'+ \
  '</firmware>\n'+ \
  '\n'+ \
  '<servos driver="Pwm">\n'+ \
    '<servo name="FRONT"   no="0" min="1000" neutral="1100" max="1900"/>\n'+ \
    '<servo name="BACK"    no="1" min="1000" neutral="1100" max="1900"/>\n'+ \
    '<servo name="RIGHT"   no="2" min="1000" neutral="1100" max="1900"/>\n'+ \
    '<servo name="LEFT"    no="3" min="1000" neutral="1100" max="1900"/>\n'+ \
  '</servos>\n'+ \
  '<commands>\n'+ \
    '<axis name="ROLL"   failsafe_value="0"/>\n'+ \
    '<axis name="PITCH"  failsafe_value="0"/>\n'+ \
    '<axis name="YAW"    failsafe_value="0"/>\n'+ \
    '<axis name="THRUST" failsafe_value="0"/>\n'+ \
  '</commands>\n'+ \
  '\n'+ \
  '<section name="MIXING" prefix="MOTOR_MIXING_">\n'+ \
    '<!-- front (CW), right (CCW), back (CW), left (CCW) -->\n'+ \
    '<define name="TYPE" value="QUAD_PLUS"/>\n'+ \
  '</section>\n'+ \
  '\n'+ \
  '<command_laws>\n'+ \
    '<call fun="motor_mixing_run(autopilot_get_motors_on(),FALSE,values)"/>\n'+ \
    '<set servo="FRONT" value="motor_mixing.commands[MOTOR_FRONT]"/>\n'+ \
    '<set servo="RIGHT" value="motor_mixing.commands[MOTOR_RIGHT]"/>\n'+ \
    '<set servo="BACK"  value="motor_mixing.commands[MOTOR_BACK]"/>\n'+ \
    '<set servo="LEFT"  value="motor_mixing.commands[MOTOR_LEFT]"/>\n'+ \
  '</command_laws>\n'+ \
  '\n'+ \
  '<section name="IMU" prefix="IMU_">\n'+ \
    '<define name="ACCEL_X_NEUTRAL" value="11"/>\n'+ \
    '<define name="ACCEL_Y_NEUTRAL" value="11"/>\n'+ \
    '<define name="ACCEL_Z_NEUTRAL" value="-25"/>\n'+ \
    '\n'+ \
    '<!-- replace this with your own calibration -->\n'+ \
    '<define name="MAG_X_NEUTRAL" value="-179"/>\n'+ \
    '<define name="MAG_Y_NEUTRAL" value="-21"/>\n'+ \
    '<define name="MAG_Z_NEUTRAL" value="79"/>\n'+ \
    '<define name="MAG_X_SENS" value="4.17334785618" integer="16"/>\n'+ \
    '<define name="MAG_Y_SENS" value="3.98885954135" integer="16"/>\n'+ \
    '<define name="MAG_Z_SENS" value="4.40442339014" integer="16"/>\n'+ \
    '\n'+ \
    '<define name="BODY_TO_IMU_PHI"   value="0." unit="deg"/>\n'+ \
    '<define name="BODY_TO_IMU_THETA" value="0." unit="deg"/>\n'+ \
    '<define name="BODY_TO_IMU_PSI"   value="0." unit="deg"/>\n'+ \
  '</section>\n'+ \
  '\n'+ \
  '<section name="AHRS" prefix="AHRS_">\n'+ \
    '<!-- values used if no GPS fix, on 3D fix is update by geo_mag module -->\n'+ \
    '<!-- Toulouse -->\n'+ \
    '<define name="H_X" value="0.513081"/>\n'+ \
    '<define name="H_Y" value="-0.00242783"/>\n'+ \
    '<define name="H_Z" value="0.858336"/>\n'+ \
  '</section>\n'+ \
  '\n'+ \
  '<section name="INS" prefix="INS_">\n'+ \
  '</section>\n'+ \
  '\n'+ \
  '<section name="STABILIZATION_ATTITUDE" prefix="STABILIZATION_ATTITUDE_">\n'+ \
    '<!-- setpoints -->\n'+ \
    '<define name="SP_MAX_PHI"     value="45." unit="deg"/>\n'+ \
    '<define name="SP_MAX_THETA"   value="45." unit="deg"/>\n'+ \
    '<define name="SP_MAX_R"       value="90." unit="deg/s"/>\n'+ \
    '<define name="DEADBAND_A"     value="0"/>\n'+ \
    '<define name="DEADBAND_E"     value="0"/>\n'+ \
    '<define name="DEADBAND_R"     value="250"/>\n'+ \
    '<!-- reference -->\n'+ \
    '<define name="REF_OMEGA_P"  value="400" unit="deg/s"/>\n'+ \
    '<define name="REF_ZETA_P"   value="0.85"/>\n'+ \
    '<define name="REF_MAX_P"    value="400." unit="deg/s"/>\n'+ \
    '<define name="REF_MAX_PDOT" value="RadOfDeg(8000.)"/>\n'+ \
    '<define name="REF_OMEGA_Q"  value="400" unit="deg/s"/>\n'+ \
    '<define name="REF_ZETA_Q"   value="0.85"/>\n'+ \
    '<define name="REF_MAX_Q"    value="400." unit="deg/s"/>\n'+ \
    '<define name="REF_MAX_QDOT" value="RadOfDeg(8000.)"/>  \n'+ \
    '<define name="REF_OMEGA_R"  value="250" unit="deg/s"/>\n'+ \
    '<define name="REF_ZETA_R"   value="0.85"/>\n'+ \
    '<define name="REF_MAX_R"    value="180." unit="deg/s"/>\n'+ \
    '<define name="REF_MAX_RDOT" value="RadOfDeg(1800.)"/>\n'+ \
    '\n'+ \
    '<!-- feedback -->\n'+ \
    '<define name="PHI_PGAIN"  value="1000"/>\n'+ \
    '<define name="PHI_DGAIN"  value="400"/>\n'+ \
    '<define name="PHI_IGAIN"  value="200"/>\n'+ \
    '\n'+ \
     '<define name="THETA_PGAIN"  value="1000"/>\n'+ \
     '<define name="THETA_DGAIN"  value="400"/>\n'+ \
     '<define name="THETA_IGAIN"  value="200"/>\n'+ \
    '\n'+ \
    '<define name="PSI_PGAIN"  value="500"/>\n'+ \
    '<define name="PSI_DGAIN"  value="300"/>\n'+ \
    '<define name="PSI_IGAIN"  value="10"/>\n'+ \
    '\n'+ \
    '<!-- feedforward -->\n'+ \
    '<define name="PHI_DDGAIN"   value="300"/>\n'+ \
    '<define name="THETA_DDGAIN" value="300"/>\n'+ \
    '<define name="PSI_DDGAIN"   value="300"/>\n'+ \
  '</section>\n'+ \
  '\n'+ \
  '<section name="GUIDANCE_V" prefix="GUIDANCE_V_">\n'+ \
    '<define name="HOVER_KP"    value="150"/>\n'+ \
    '<define name="HOVER_KD"    value="80"/>\n'+ \
    '<define name="HOVER_KI"    value="20"/>\n'+ \
    '<define name="NOMINAL_HOVER_THROTTLE" value="0.5"/>\n'+ \
    '<define name="ADAPT_THROTTLE_ENABLED" value="TRUE"/>\n'+ \
  '</section>\n'+ \
  '\n'+ \
  '<section name="GUIDANCE_H" prefix="GUIDANCE_H_">\n'+ \
    '<define name="MAX_BANK" value="20" unit="deg"/>\n'+ \
    '<define name="USE_SPEED_REF" value="TRUE"/>\n'+ \
    '<define name="PGAIN" value="50"/>\n'+ \
    '<define name="DGAIN" value="100"/>\n'+ \
    '<define name="AGAIN" value="70"/>\n'+ \
    '<define name="IGAIN" value="20"/>\n'+ \
  '</section>\n'+ \
  '\n'+ \
  '<section name="NAV">\n'+ \
  '  <define name="ARRIVED_AT_WAYPOINT" value="2" unit="m"/>\n'+ \
  '</section>\n'+ \
  '\n'+ \
  '<section name="SIMULATOR" prefix="NPS_">\n'+ \
  '  <define name="ACTUATOR_NAMES"  value="front_motor, right_motor, back_motor, left_motor" type="string[]"/>\n'+ \
  '  <define name="JSBSIM_MODEL" value="simple_quad" type="string"/>\n'+ \
  '  <define name="SENSORS_PARAMS" value="nps_sensors_params_default.h" type="string"/>\n'+ \
  '  <!-- mode switch on joystick channel 5 (axis numbering starting at zero) -->\n'+ \
  '  <define name="JS_AXIS_MODE" value="4"/>\n'+ \
  '</section>\n'+ \
  '\n'+ \
  '<section name="AUTOPILOT">\n'+ \
  '  <define name="MODE_MANUAL" value="AP_MODE_ATTITUDE_DIRECT"/>\n'+ \
  '  <define name="MODE_AUTO1"  value="AP_MODE_HOVER_Z_HOLD"/>\n'+ \
  '  <define name="MODE_AUTO2"  value="AP_MODE_NAV"/>\n'+ \
  '</section>\n'+ \
  '\n'+ \
  '<section name="BAT">\n'+ \
  '  <define name="CATASTROPHIC_BAT_LEVEL" value="9.3" unit="V"/>\n'+ \
  '  <define name="CRITIC_BAT_LEVEL" value="9.6" unit="V"/>\n'+ \
  '  <define name="LOW_BAT_LEVEL" value="10.1" unit="V"/>\n'+ \
  '  <define name="MAX_BAT_LEVEL" value="12.4" unit="V"/>\n'+ \
  '  <define name="MILLIAMP_AT_FULL_THROTTLE" value="30000"/>\n'+ \
  '</section>\n'+ \
'</airframe>'


def setup_copters(fn:str):
  with open((SCENARIO_DIR+'/'+fn),"r") as inputFile:
    lines = inputFile.readlines()
    values = lines[0].strip().split(",")
    amounts = {}
    for val in values: amounts[val.strip().split(":")[0]] = int(val.strip().split(":")[-1])
    if(len(lines[2:])==amounts['spawn']):
      for ac_id,coords in enumerate(lines[2:]):
        with open((FLIGHT_PLAN_DIR+("/member%d.xml"%ac_id)),"w") as outputFile:
          coords = coords.strip().split(",")
          coords = ('lat0="'+coords[0].strip()+'" lon0="'+coords[1].strip()+'"')
          waypoints=''
          for att_id in range(amounts["att"]): waypoints+=('<waypoint name="ATTRECTION_POINT_%d" x="0.0" y="0.0"/>\n'%att_id)
          for rep_id in range(amounts["rep"]): waypoints+=('<waypoint name="REPELL_POINT_%d" x="0.0" y="0.0"/>\n'%rep_id)  
          outputFile.write((FLIGHT_PLAN_HEADER+coords+FLIGHT_PLAN_BODY+waypoints+FLIGHT_PLAN_END))
      with open((COPTER_DIR+"/npsSwarmCopter.xml"),"w") as outputFile:
        waypoints=''
        waypoints+='<define name="FIRST_ATTRACTION_POINT_ID" value="2"/>' 
        waypoints+=('<define name="LAST_ATTRACTION_POINT_ID" value="%d"/>'%(2+amounts["att"]))  
        waypoints+=('<define name="FIRST_REPELL_POINT_ID" value="%d"/>'%(2+amounts["att"])) 
        waypoints+=('<define name="LAST_REPELL_POINT_ID" value="%d"/>'%(2+amounts["att"]+amounts["rep"])) 
        outputFile.write((COPTER_HEADER+waypoints+COPTER_END))


#main program
if __name__=='__main__':
    if len(sys.argv)>1:
      setup_copters(sys.argv[1])