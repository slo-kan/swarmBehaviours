#!/usr/bin/python
import sys,time
from os import path,getenv

PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME+ "/var/lib/python")

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage


#global vars
ATT_POINTS = [[52.13878, 11.64539], [52.1389, 11.64589], [52.13899, 11.64465], [52.13843, 11.64448], [52.13829, 11.64588]]
REP_POINTS = [[52.139193227158565, 11.645442243819168], [52.13873185661875, 11.64604126781022], [52.13983556877823, 11.647903288820826], [52.13927000303959, 11.6465389796281], [52.13972261306308, 11.644770246818533]]
START_ID =  30
END_ID = 40
ATT_ID = 2
REP_ID = 3
ALT = 47

moveWP = []
epoche = 0
counter = 1

interface = IvyMessagesInterface(
            agent_name="Pprzlink_Move_WP",      # Ivy agent name
            start_ivy=False,                    # Do not start the ivy bus now
            ivy_bus="127.255.255.255:2010")     # address of the ivy bus



def recv_callback(ac_id, recvMsg):
    #out = open("/home/finkensim/finken/paparazzi/sw/tools/ovgu_swarm/WP_Mover.log","a")
    #out.write("%d. MSG: %s\n" % (counter, recvMsg))
    #out.close()
    #counter+=1
    global epoche,moveWP,interface

    if(int(recvMsg.__getitem__("achieved"))>0):
        if(abs(int(ATT_POINTS[epoche%len(ATT_POINTS)][0]*1e7)-int(recvMsg.__getitem__("lat")))<5 and 
           abs(int(ATT_POINTS[epoche%len(ATT_POINTS)][1]*1e7)-int(recvMsg.__getitem__("lon")))<5):

            epoche += 1    
            moveWP = []

            for AC_ID in range(START_ID, END_ID):
                moveWP.append(createMSG(int(ATT_ID),int(AC_ID),int(ATT_POINTS[epoche%len(ATT_POINTS)][0]*1e7),
                    int(ATT_POINTS[epoche%len(ATT_POINTS)][1]*1e7),int(ALT)))
                moveWP.append(createMSG(int(REP_ID),int(AC_ID),int(REP_POINTS[epoche%len(REP_POINTS)][0]*1e7),
                    int(REP_POINTS[epoche%len(REP_POINTS)][1]*1e7),int(ALT)))
            out = open("/home/finkensim/finken/paparazzi/sw/tools/ovgu_swarm/WP_Mover.log","a")
            for i,msg in enumerate(moveWP):
                interface.send(msg)
                out.write("%d. MSG: %s\n" % (i, msg))
            out.close()

            
def createMSG(wp_id,ac_id,lat,lon,alt):
    msg = PprzMessage("datalink", "MOVE_WP")
    msg.set_value_by_name("wp_id", wp_id)
    msg.set_value_by_name("ac_id", ac_id)
    msg.set_value_by_name("lat", lat)
    msg.set_value_by_name("lon", lon)
    msg.set_value_by_name("alt", alt)
    return msg

def main():
    global moveWP,interface

    #init vars
    for AC_ID in range(START_ID, END_ID):
        moveWP.append(createMSG(int(ATT_ID),int(AC_ID),int(ATT_POINTS[0][0]*1e7),int(ATT_POINTS[0][1]*1e7),int(ALT)))
        moveWP.append(createMSG(int(REP_ID),int(AC_ID),int(REP_POINTS[0][0]*1e7),int(REP_POINTS[0][1]*1e7),int(ALT)))    
    out = open("/home/finkensim/finken/paparazzi/sw/tools/ovgu_swarm/WP_Mover.log","a")
    out.write("Start_ID: "+str(START_ID)+"; End_ID: "+str(END_ID)+"; moveWP length: "+str(len(moveWP))+"\n")
    out.close()

    #run program
    try:
        interface.start()
        time.sleep(3)
        interface.subscribe(recv_callback,PprzMessage("telemetry", "GOAL_ACHIEVED"))
        out = open("/home/finkensim/finken/paparazzi/sw/tools/ovgu_swarm/WP_Mover.log","a")
        for i,msg in enumerate(moveWP):
            interface.send(msg)
            out.write("%d. MSG: %s\n" % (i, msg))
        out.close()
        while True: 
            time.sleep(10)
    except:
        interface.shutdown()


if __name__=='__main__':
    out = open("/home/finkensim/finken/paparazzi/sw/tools/ovgu_swarm/WP_Mover.log","w")
    out.write("Start FlightPlan...\n")
    out.close()
    main()
