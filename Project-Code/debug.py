#!/usr/bin/python

## @file debug.py
#  @brief An algorithm to debug the wp_mover.py and to check if it is working the way it is expected to.

## @brief module imports
import sys,time
from os import path,getenv

## @brief add PPRZ_HOME var to Path
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME+ "/var/lib/python")

## @brief import pprzlink interface components
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

## @brief Function called when a PING message is received and print the message and the sender id
#  @param ac_is aircraft id
#  @param recvMsg is the teemetry message consisting of waypoint id, its coordiantes. 
#  Can be found in messages.xml with the message name "GOAL_ACHIEVED" aad id "76"
def recv_callback(ac_id, pprzMsg):
    out = open("/home/finkensim/finken/paparazzi/sw/tools/ovgu_swarm/WP_Mover.debug","a")
    out.write("Received message %s from %s\n" % (pprzMsg,ac_id))
    out.close()

def main():
    ## Creation of the ivy interface
    ivy = IvyMessagesInterface(
                ## agent_name Ivy agent name
                agent_name="Debug_Move_WP",
                ## start_ivy is a boolean which if false do not start the ivy bus now
                start_ivy=False,
                ##ivy_bus is the address of the ivy bus                 
                ivy_bus="127.255.255.255:2010") 

    try:
        ## starts the ivy interface
        ivy.start()

        ## Subscribe to PING messages and sets recv_callback as the callback function.
        ivy.subscribe(recv_callback,PprzMessage("ground", "MOVE_WAYPOINT"))

        ## Wait untill terminated
        while True:
            time.sleep(10)
    except:
        ivy.shutdown()

## @brief main program
if __name__=='__main__':
    time.sleep(10)
    out = open("/home/finkensim/finken/paparazzi/sw/tools/ovgu_swarm/WP_Mover.debug","w")
    out.write("Start Debugger...\n")
    out.close()
    main()



