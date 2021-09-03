#!/usr/bin/python

#module imports
import sys,time
from os import path,getenv

#add PPRZ_HOME var to Path
DIR = path.dirname(path.abspath(__file__))
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(DIR, '../../../../')))
sys.path.append(PPRZ_HOME+ "/var/lib/python")

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

STANDARD_ALT = 47


#####################
# Class Definitions #
#####################
# Point defined as {lat,lon,alt} or {x,y,z}
class Point:
    idxs = {0: 0, 1: 1, 2: 2, 'lat': 0, 'lon': 1, 'alt': 2, 'x': 0, 'y': 1, 'z': 2}

    def __init__(self,x,y,z=STANDARD_ALT):
        self.coords = [x,y,z]

    def __getitem__(self,idx):
        return self.coords[self.idxs[idx]]
    
    def __setitem__(self,idx,val):
        self.coords[self.idxs[idx]] = val


#Input Function
#scan file to create flight plan
def get_szenario(filename):
    att_points,rep_points = ([],[])
    file = open((DIR+"/"+filename),"r")
    line = file.readline()
    points = line.strip().split(";")
    for point in points:
        coords = point.strip().split(",")
        coords = list(map(lambda coord: int(float(coord)*1e7),coords))
        if len(coords)==3: att_points.append(Point(coords[0],coords[1],coords[2]))
        else: att_points.append(Point(coords[0],coords[1]))
    line = file.readline()
    points = line.strip().split(";")
    for point in points:
        coords = point.strip().split(",")
        coords = list(map(lambda coord: int(float(coord)*1e7),coords))
        if len(coords)==3: rep_points.append(Point(coords[0],coords[1],coords[2]))
        else: rep_points.append(Point(coords[0],coords[1]))
    return (att_points,rep_points)



#global constants
ATT_POINTS, REP_POINTS = get_szenario("flight_plan.cvs")
START_ID, END_ID = (30,40)
ATT_ID, REP_ID = (2,3)

#global variables
sending   = False
updating  = False
moveWP    = []
epoche    = 0
counter   = 1
interface = IvyMessagesInterface(
            agent_name="Pprzlink_Move_WP",      # Ivy agent name
            start_ivy=False,                    # Do not start the ivy bus now
            ivy_bus="127.255.255.255:2010")     # address of the ivy bus




########################
# Function Definitions #
########################

#create PprzMessage            
def createMSG(wp_id,ac_id,point):
    msg = PprzMessage("datalink", "MOVE_WP")
    msg.set_value_by_name("wp_id", wp_id)
    msg.set_value_by_name("ac_id", ac_id)
    msg.set_value_by_name("lat", point[0])
    msg.set_value_by_name("lon", point[1])
    msg.set_value_by_name("alt", point[2])
    return msg


#send update msgs through interface
def send_msgs():
    global moveWP,interface, updating, sending

    #out = open("/home/finkensim/finken/paparazzi/sw/tools/ovgu_swarm/WP_Mover.log","a")
    sending = True
    while(not updating):
        for n in range(2):
            for i,msg in enumerate(moveWP):
                time.sleep(0.1)
                interface.send(msg)
    #            out.write("%d. WP_Move-MSG: %s\n" % (i, msg))
    #out.close()
    sending = False


#manipulate goal points via msg updates
def recv_callback(ac_id, recvMsg):
    global epoche,moveWP,counter, updating, sending

    if(int(recvMsg["achieved"])>0):
        #out = open("/home/finkensim/finken/paparazzi/sw/tools/ovgu_swarm/WP_Mover.log","a")
        #out.write("%d. Goal_Achieved-MSG: %s\n" % (counter, recvMsg))
        #out.close()
        #counter+=1

        if(abs(ATT_POINTS[epoche%len(ATT_POINTS)]["lat"]-int(recvMsg["lat"]))<500 and 
           abs(ATT_POINTS[epoche%len(ATT_POINTS)]["lon"]-int(recvMsg["lon"]))<500):
            updating = True
            epoche += 1
            #out = open("/home/finkensim/finken/paparazzi/sw/tools/ovgu_swarm/WP_Mover.log","a")
            #out.write("%d. Epoche: \n" % epoche)
            #out.close()    
            moveWP = []
            for AC_ID in range(START_ID, END_ID):
                moveWP.append(createMSG(int(ATT_ID),int(AC_ID),ATT_POINTS[epoche%len(ATT_POINTS)]))
                moveWP.append(createMSG(int(REP_ID),int(AC_ID),REP_POINTS[epoche%len(REP_POINTS)]))
            updating = False
            send_msgs()
            

#main method
def main():
    global moveWP,interface

    #init vars
    for AC_ID in range(START_ID, END_ID):
        moveWP.append(createMSG(int(ATT_ID),int(AC_ID),ATT_POINTS[0]))
        moveWP.append(createMSG(int(REP_ID),int(AC_ID),REP_POINTS[0]))    
    out = open("/home/finkensim/finken/paparazzi/sw/tools/ovgu_swarm/WP_Mover.log","a")
    out.write("Start_ID: "+str(START_ID)+"; End_ID: "+str(END_ID)+"; moveWP length: "+str(len(moveWP))+"\n")
    out.close()

    #run program
    try:
        interface.start()
        interface.subscribe(recv_callback,PprzMessage("telemetry", "GOAL_ACHIEVED"))
        send_msgs()
        while True: 
            time.sleep(10)
    except:
        interface.shutdown()


#main program
if __name__=='__main__':
    out = open("/home/finkensim/finken/paparazzi/sw/tools/ovgu_swarm/WP_Mover.log","w")
    out.write("Start FlightPlan...\n")
    out.close()
    main()
