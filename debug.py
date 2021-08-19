#!/usr/bin/python

import sys,time
from os import path,getenv

PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME+ "/var/lib/python")

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

# Function called when a PING message is received
def recv_callback(ac_id, pprzMsg):
    out = open("/home/finkensim/finken/paparazzi/sw/tools/ovgu_swarm/WP_Mover.debug","a")
    # Print the message and the sender id
    out.write("Received message %s from %s\n" % (pprzMsg,ac_id))
    out.close()

def main():
    # Creation of the ivy interface
    ivy = IvyMessagesInterface(
                agent_name="Debug_Move_WP",         # Ivy agent name
                start_ivy=False,                    # Do not start the ivy bus now
                ivy_bus="127.255.255.255:2010")     # address of the ivy bus

    try:
        # starts the ivy interface
        ivy.start()

        # Subscribe to PING messages and sets recv_callback as the callback function.
        ivy.subscribe(recv_callback,PprzMessage("datalink", "MOVE_WP"))

        # Wait untill ^C is pressed
        while True:
            time.sleep(10)
    except:
        ivy.shutdown()

if __name__=='__main__':
    time.sleep(10)
    out = open("/home/finkensim/finken/paparazzi/sw/tools/ovgu_swarm/WP_Mover.debug","w")
    out.write("Start Debugger...\n")
    out.close()
    main()



