#!/usr/bin/python
import sys,time
from os import path,getenv

PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME+ "/var/lib/python")

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

START_ID = 42 
END_ID = 46

def createMSG(wp_id,ac_id,lat,lon,alt):
    msg = PprzMessage("datalink", "MOVE_WP")
    msg.set_value_by_name("wp_id", wp_id)
    msg.set_value_by_name("ac_id", ac_id)
    msg.set_value_by_name("lat", lat)
    msg.set_value_by_name("lon", lon)
    msg.set_value_by_name("alt", alt)
    return msg

def main():
    interface = IvyMessagesInterface(
            agent_name="Pprzlink_Move_WP",      # Ivy agent name
            start_ivy=False,                    # Do not start the ivy bus now
            ivy_bus="127.255.255.255:2010")     # address of the ivy bus
    offset = 0.0001
    moveWP = []
    for AC_ID in range(START_ID, END_ID):
        moveWP.append(createMSG(int(2),int(AC_ID),int((52.13785+(offset*(AC_ID-START_ID)))*1e7),int(11.64540*1e7),int(47)))
    out = open("/home/finkensim/finken/paparazzi/sw/tools/ovgu_swarm/WP_Mover.log","a")
    out.write("Start_ID: "+str(START_ID)+"; End_ID: "+str(END_ID)+"; moveWP length: "+str(len(moveWP))+"\n")
    out.close()
    try:
        interface.start()
        time.sleep(5)
        for i,msg in enumerate(moveWP):
            interface.send(msg)
            out = open("/home/finkensim/finken/paparazzi/sw/tools/ovgu_swarm/WP_Mover.log","a")
            out.write("%d. MSG: %s\n" % (i, msg))
            out.close()
        while True: 
            time.sleep(10)
    except:
        interface.shutdown()

if __name__=='__main__':
    time.sleep(12)
    out = open("/home/finkensim/finken/paparazzi/sw/tools/ovgu_swarm/WP_Mover.log","w")
    out.write("Start WPMover...\n")
    out.close()
    main()