#!/usr/bin/python

#module imports
import sys,time,math
from threading import Thread
from os import path,getenv

#add PPRZ_HOME var to Path
DIR = path.dirname(path.abspath(__file__))
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(DIR, '../../../../')))
sys.path.append(PPRZ_HOME+ "/var/lib/python")

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

#####################
# Class Definitions #
#####################
# Point defined as {lat,lon,alt} or {x,y,z}
class Point:
    __IDXS = {0: 0, 1: 1, 2: 2, 'lat': 0, 'lon': 1, 'alt': 2, 'x': 0, 'y': 1, 'z': 2}
    __STANDARD_ALT = float(47)

    def __init__(self,x,y,z=float(__STANDARD_ALT)):
        self.coords = [float(x),float(y),float(z)]

    def __getitem__(self,idx):
        return float(self.coords[self.__IDXS[idx]])
    
    def __setitem__(self,idx,val):
        self.coords[self.__IDXS[idx]] = float(val)


#Input Function
#scan file to create flight plan
def get_szenario(filename):
    att_points,rep_points = ([],[])
    file = open((DIR+"/"+filename),"r")
    line = file.readline()
    points = line.strip().split(";")
    for point in points:
        coords = point.strip().split(",")
        coords = list(map(lambda coord: (float(coord)*math.pi/180),coords))
        if len(coords)==3: att_points.append(Point(coords[0],coords[1],coords[2]))
        else: att_points.append(Point(coords[0],coords[1]))
    line = file.readline()
    points = line.strip().split(";")
    for point in points:
        coords = point.strip().split(",")
        coords = list(map(lambda coord: (float(coord)*math.pi/180),coords))
        if len(coords)==3: rep_points.append(Point(coords[0],coords[1],coords[2]))
        else: rep_points.append(Point(coords[0],coords[1]))
    return (att_points,rep_points)


#global constants
GLOBE_RADIUS = 6371000
ATT_POINTS, REP_POINTS = get_szenario("flight_plan.cvs")
START_ID, END_ID = (int(30),int(40))                #end id is exclusiv 40 means last copter in the swarm has id 39
ATT_ID, REP_ID = (int(2),int(3))
INTERFACE = IvyMessagesInterface(
                agent_name="Pprzlink_Move_WP",      # Ivy agent name
                start_ivy=False,                    # Do not start the ivy bus now
                ivy_bus="127.255.255.255:2010"      # address of the ivy bus
            )

#global variables
sendingThread = None
updating      = False
moveWP        = []
epoche        = 0
#counter       = 1





########################
# Function Definitions #
########################

#create PprzMessage            
def createMSG(wp_id,ac_id,point):
    msg = PprzMessage("datalink", "MOVE_WP")
    msg.set_value_by_name("wp_id", wp_id)
    msg.set_value_by_name("ac_id", ac_id)
    msg.set_value_by_name("lat", point["lat"])
    msg.set_value_by_name("lon", point["lon"])
    msg.set_value_by_name("alt", point["alt"])
    return msg


#send update msgs through interface
def send_msgs():
    global moveWP, updating

    #if not updating:
    #    out = open("/home/finkensim/finken/paparazzi/sw/tools/ovgu_swarm/WP_Mover.log","a")
    while(True):
        string = ""
        for _ in range(2):
            for i,msg in enumerate(moveWP):
                time.sleep(0.1)
                INTERFACE.send(msg)
                string += ("%d. WP_Move-MSG: %s\n" % (i, msg))
        #out.write(string)
        if updating: break
        time.sleep(1)
    #out.close()


#function to create and start new thread
def reset_thread():
    global sendingThread

    sendingThread = Thread(target=send_msgs,name="Send_Msg_Thread")
    sendingThread.start()

#gets the metric distance between two points given in the LlaCoor_f format
def getDistance(own_pos, goal_pos):
    return (GLOBE_RADIUS * math.acos(                  
    math.sin(own_pos["lat"])*math.sin(goal_pos["lat"]) +    
    math.cos(own_pos["lat"])*math.cos(goal_pos["lat"]) *    
    math.cos(own_pos["lon"] - goal_pos["lon"])))


#manipulate goal points via msg updates
def recv_callback(ac_id, recvMsg):
    global epoche, moveWP, updating, sendingThread#, counter

    if(int(recvMsg["achieved"])>0):
        #out = open("/home/finkensim/finken/paparazzi/sw/tools/ovgu_swarm/WP_Mover.log","a")
        #out.write("%d. Goal_Achieved-MSG: %s\n" % (counter, recvMsg))
        #out.close()
        #counter+=1
        own_pos = Point(float(recvMsg["lat"]),float(recvMsg["lon"]),float(recvMsg["alt"]))

        if(getDistance(own_pos,ATT_POINTS[epoche%len(ATT_POINTS)])<=1.5):
            updating = True
            sendingThread.join()

            epoche += 1
            moveWP = []
            out = open("/home/finkensim/finken/paparazzi/sw/tools/ovgu_swarm/WP_Mover.log","a")
            out.write("%d. Epoche: \n" % epoche)
            out.close()

            for ac_ID in range(START_ID, END_ID):
                moveWP.append(createMSG(ATT_ID,ac_ID,ATT_POINTS[epoche%len(ATT_POINTS)]))
                moveWP.append(createMSG(REP_ID,ac_ID,REP_POINTS[epoche%len(REP_POINTS)]))
            updating = False
            reset_thread()
            

#main method
def main():
    #out = open("/home/finkensim/finken/paparazzi/sw/tools/ovgu_swarm/WP_Mover.log","a")
    #out.write("First Att_Point: %d, %d, %d\n" % (ATT_POINTS[0]["lat"], ATT_POINTS[0]["lon"], ATT_POINTS[0]["alt"]))
    #out.close()

    global moveWP, sendingThread

    #init vars
    for ac_ID in range(START_ID, END_ID):
        moveWP.append(createMSG(ATT_ID,ac_ID,ATT_POINTS[0]))
        moveWP.append(createMSG(REP_ID,ac_ID,REP_POINTS[0]))    
    out = open("/home/finkensim/finken/paparazzi/sw/tools/ovgu_swarm/WP_Mover.log","a")
    out.write("Start_ID: "+str(START_ID)+"; End_ID: "+str(END_ID)+"; moveWP length: "+str(len(moveWP))+"\n")
    out.close()

    #run program
    try:
        INTERFACE.start()
        INTERFACE.subscribe(recv_callback,PprzMessage("telemetry", "GOAL_ACHIEVED"))
        reset_thread()
        while True: 
            time.sleep(10)
    except:
        updating = True
        sendingThread.join()
        INTERFACE.shutdown()
        


#main program
if __name__=='__main__':
    out = open("/home/finkensim/finken/paparazzi/sw/tools/ovgu_swarm/WP_Mover.log","w")
    out.write("Start FlightPlan...\n")
    out.close()
    main()
