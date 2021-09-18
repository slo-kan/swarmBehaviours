#!/usr/bin/python

#module imports
import sys,time,math
from threading import Thread, Lock
from queue import Queue,Empty
from os import path,getenv

#add PPRZ_HOME var to Path
DIR = path.dirname(path.abspath(__file__))
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(DIR, '../../../../')))
sys.path.append(PPRZ_HOME+ "/var/lib/python")

#import pprzlink interface components
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage



#####################
# Class Definitions #
#####################
# Point coordinates can be accesed by {lat,lon,alt}, {x,y,z} or {0,1,2}
class Point:
    @property
    def __IDXS(self): return {0: 0, 1: 1, 2: 2, 'lat': 0, 'lon': 1, 'alt': 2, 'x': 0, 'y': 1, 'z': 2}
    __STANDARD_ALT:float = 47.0

    def __init__(self,x:float,y:float,z:float=__STANDARD_ALT):
        self.__coords = [x,y,z]

    def __getitem__(self,idx)->float:
        return float(self.__coords[self.__IDXS[idx]])
    
    def __setitem__(self,idx,val:float):
        self.__coords[self.__IDXS[idx]] = val


# Simple but thread safe logger
class ThreadSafe_Logger:
    __QUEUE:Queue = Queue()
    __finished:bool = False

    def __write_to_file(self):
        while not self.__finished or not self.__QUEUE.empty():
            try: data = self.__QUEUE.get(timeout=0.2)
            except Empty: time.sleep(1)
            else: 
                self.__FILE_WRITER.write(data)
                self.__QUEUE.task_done()

    def __init__(self, filename):
        self.__FILENAME = filename
        self.__WORKER = Thread(target=self.__write_to_file,name="Logging_Thread")
    
    def start(self): 
        self.__FILE_WRITER = open(self.__FILENAME,"a")
        self.__WORKER.start()
        self.write("Logging_Thread started...\n")

    def write(self, data:str): self.__QUEUE.put(data)
    def close(self):
        self.write("... Logging_Thread closed!\n")
        self.__finished = True
        self.__QUEUE.join()
        self.__WORKER.join()
        self.__FILE_WRITER.close() 



#Input Function
#scan file to create flight plan
def get_szenario(filename)->"tuple[list[Point]]":
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
LOGGER = ThreadSafe_Logger("/home/finkensim/finken/paparazzi/sw/tools/ovgu_swarm/WP_Mover.log")
ATT_POINTS, REP_POINTS = get_szenario("flight_plan.cvs")
START_ID, END_ID = (int(30),int(40))                #end id is exclusiv 40 means last copter in the swarm has id 39
ATT_ID, REP_ID = (int(2),int(3))
MSG_ACCESS:Lock = Lock()
GLOBE_RADIUS:int = int(6371000)
INTERFACE = IvyMessagesInterface(
                agent_name="Pprzlink_Move_WP",      # Ivy agent name
                start_ivy=False,                    # Do not start the ivy bus now
                ivy_bus="127.255.255.255:2010"      # address of the ivy bus
            )

#global variables
moveWP:"list[PprzMessage]" = []
terminate :bool   = False
epoche    :int    = 0
#counter  :int    = 1





########################
# Function Definitions #
########################

#converts a LLA-Point from float radial representation to a int degree representation
def convertToInt(point:Point)->"tuple[int]":
    lat = int((point["lat"]*180/math.pi)*1e7)
    lon = int((point["lon"]*180/math.pi)*1e7)
    alt = int(point["alt"])
    return (lat,lon,alt)


#converts a LLA-Point from float radial representation to float degree representation
def convertToDegree(point:Point)->"tuple[float]":
    lat = float(point["lat"]*180/math.pi)
    lon = float(point["lon"]*180/math.pi)
    alt = float(point["alt"])
    return (lat,lon,alt)


#create PprzMessage            
def createMSG(wp_id:int,ac_id:int,point:Point)->"PprzMessage":
    msg = PprzMessage("ground", "MOVE_WAYPOINT")
    msg['wp_id'] = wp_id
    msg['ac_id'] = str(ac_id)
    msg['lat'],msg['lon'],msg['alt'] = convertToDegree(point)
    return msg


#send update msgs through interface
def send_msgs():
    global moveWP, terminate
    
    LOGGER.write("MSG-Thread started ... \n")
    while not terminate:
        aquired = MSG_ACCESS.acquire(timeout=2.0)
        if aquired:
            LOGGER.write("len(moveWP) = %d\n" % len(moveWP))
            string = ""
            for i,msg in enumerate(moveWP):
                time.sleep(0.2)
                INTERFACE.send(msg)
                pre = str("ATT" if ATT_ID == msg['wp_id'] else "REP")
                string += ("%2d. %3s-WP_Move-MSG: %s\n" % (i, pre, msg))
            LOGGER.write(string)
            MSG_ACCESS.release()
            time.sleep(3)
    LOGGER.write("... MSG-Thread closed! \n")

#create constant global MSG_SENDING_THREAD 
MSG_SENDING_THREAD = Thread(target=send_msgs,name="Send_Msg_Thread")


#gets the metric distance between two points given in the LlaCoor_f format
def getDistance(own_pos:Point, goal_pos:Point)->float:
    return float( GLOBE_RADIUS * math.acos(                  
        math.sin(own_pos["lat"]) * math.sin(goal_pos["lat"]) +    
        math.cos(own_pos["lat"]) * math.cos(goal_pos["lat"]) *    
        math.cos(own_pos["lon"] - goal_pos["lon"])
    ))


#manipulate goal points via msg updates
def recv_callback(ac_id, recvMsg):
    global epoche, moveWP#, counter

    if(int(recvMsg["achieved"])>0):
        own_pos = Point(float(recvMsg["lat"]),float(recvMsg["lon"]),float(recvMsg["alt"]))
        #out = open("/home/finkensim/finken/paparazzi/sw/tools/ovgu_swarm/WP_Mover.log","a")
        #out.write("%d. Goal_Achieved-MSG: %s\n" % (counter, recvMsg))
        #out.close()
        #counter+=1

        if(getDistance(own_pos,ATT_POINTS[epoche%len(ATT_POINTS)])<=1.5):
            aquired = MSG_ACCESS.acquire(timeout=3.0)
            if aquired:
                epoche += 1
                moveWP = []
                for acId in range(START_ID, END_ID):
                    moveWP.append(createMSG(ATT_ID,acId,ATT_POINTS[epoche%len(ATT_POINTS)]))
                    moveWP.append(createMSG(REP_ID,acId,REP_POINTS[epoche%len(REP_POINTS)]))
                LOGGER.write("%d send this valid Goal_Achieved-MSG: %s\n" % (ac_id, recvMsg))
                LOGGER.write("%d. Epoche - " % epoche)
                MSG_ACCESS.release()
            

#main method
def main():
    global moveWP, terminate

    out = open("/home/finkensim/finken/paparazzi/sw/tools/ovgu_swarm/WP_Mover.log","w")
    out.write("Start FlightPlan...\n")
    out.close()
    time.sleep(3)

    #init vars
    LOGGER.start()
    with MSG_ACCESS:
        for acId in range(START_ID, END_ID):
            moveWP.append(createMSG(ATT_ID,acId,ATT_POINTS[0]))
            moveWP.append(createMSG(REP_ID,acId,REP_POINTS[0]))    
    LOGGER.write(str("Start_ID: "+str(START_ID)+"; End_ID: "+str(END_ID)+"; moveWP length: "+str(len(moveWP))+"\n"))
    LOGGER.write("Init-Epoche - ")

    #run program
    try:
        INTERFACE.start()
        INTERFACE.subscribe(recv_callback,PprzMessage("telemetry", "GOAL_ACHIEVED"))
        MSG_SENDING_THREAD.start()
        time.sleep(15)
        raise KeyboardInterrupt
    except KeyboardInterrupt: LOGGER.write("... !STOPPING FlightPlan! ...\n")
    finally:
        terminate = True
        MSG_SENDING_THREAD.join()
        time.sleep(3)
        LOGGER.close()
        INTERFACE.shutdown()
        


#main program
if __name__=='__main__':
    main()