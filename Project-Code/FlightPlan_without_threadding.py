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
def CURRENT_TIME()->float: return time.time()
GLOBE_RADIUS:int = int(6371000)
TIMER = 40



class WP_Mover:
    #class-global consts
    __SENDING:Lock = Lock()
    __ATT_ID, __REP_ID = (int(2),int(3))

    #class-global variables
    __moveWP:"list[PprzMessage]" = []
    __epoche :int = 0
    __counter:int = 1



    ########################
    # Function Definitions #
    ########################

    def __init__(self, start_id:int, end_id:int, filename:str, debug:bool=False):
        self.__ATT_POINTS, self.__REP_POINTS = get_szenario(filename)
        self.__START_ID = start_id  #start id is inclusiv -> 30 means first copter in the swarm has id 30
        self.__END_ID = end_id  #end id is exclusiv -> 40 means last copter in the swarm has id 39

        for acId in range(self.__START_ID, self.__END_ID):
            self.__moveWP.append(WP_Mover.__createMSG(self.__ATT_ID,acId,self.__ATT_POINTS[0]))
            self.__moveWP.append(WP_Mover.__createMSG(self.__REP_ID,acId,self.__REP_POINTS[0]))

        with open("/home/finkensim/finken/paparazzi/sw/tools/ovgu_swarm/WP_Mover.log","w") as LOGGER:
            LOGGER.write("Start FlightPlan...\n")

        self.__DEBUG = debug
        self.__INTERFACE = IvyMessagesInterface("Pprzlink_Move_WP")


    def __del__(self):
        self.__INTERFACE.unsubscribe(self.__bind_id)
        with open("/home/finkensim/finken/paparazzi/sw/tools/ovgu_swarm/WP_Mover.log","a") as LOGGER: 
            LOGGER.write("... STOPPED FlightPlan!\n")
        self.__INTERFACE.shutdown()


    #converts a LLA-Point from float radial representation to a int degree representation
    @staticmethod
    def convertToInt(point:Point)->"tuple[int]":
        lat = int((point["lat"]*180/math.pi)*1e7)
        lon = int((point["lon"]*180/math.pi)*1e7)
        alt = int(point["alt"])
        return (lat,lon,alt)
    

    #converts a LLA-Point from float radial representation to float degree representation
    @staticmethod
    def convertToDegree(point:Point)->"tuple[float]":
        lat = float(point["lat"]*180/math.pi)
        lon = float(point["lon"]*180/math.pi)
        alt = float(point["alt"])
        return (lat,lon,alt)
    

    #gets the metric distance between two points given in the LlaCoor_f format
    @staticmethod
    def getDistance(own_pos:Point, goal_pos:Point)->float:
        return float( GLOBE_RADIUS * math.acos(                  
            math.sin(own_pos["lat"]) * math.sin(goal_pos["lat"]) +    
            math.cos(own_pos["lat"]) * math.cos(goal_pos["lat"]) *    
            math.cos(own_pos["lon"] - goal_pos["lon"])
        ))


    #create PprzMessage 
    @staticmethod           
    def __createMSG(wp_id:int,ac_id:int,point:Point)->"PprzMessage":
        msg = PprzMessage("ground","MOVE_WAYPOINT")
        lat,lon,alt = WP_Mover.convertToDegree(point)
        msg.set_values([str(ac_id), wp_id, lat, lon, alt])
        return msg


    #function for sending msgs periodically
    def __send_msgs(self):
        if self.__SENDING.acquire(timeout=0.1):
            string = ""
            for i,msg in enumerate(self.__moveWP):
                time.sleep(0.01)
                self.__INTERFACE.send(msg)
                pre = str("ATT" if msg['wp_id'] == self.__ATT_ID  else "REP")
                string += ("%2d. %3s-WP_Move-MSG: %s\n" % (i, pre, msg))
            if self.__DEBUG:
                with open("/home/finkensim/finken/paparazzi/sw/tools/ovgu_swarm/WP_Mover.log","a") as LOGGER:
                    LOGGER.write("len(moveWP) = %d\n" % len(self.__moveWP))
                    LOGGER.write(string)
            self.__SENDING.release()


    #main method - executes logic
    def run(self):

        #manipulate goal points via msg updates
        def recv_callback(ac_id, recvMsg):
            change = False
            if(int(recvMsg["achieved"])>0):
                own_pos = Point(float(recvMsg["lat"]),float(recvMsg["lon"]),float(recvMsg["alt"]))
                if(WP_Mover.getDistance(own_pos,self.__ATT_POINTS[self.__epoche%len(self.__ATT_POINTS)])<=1.5):
                    change = True
                    self.__epoche += 1
                    with open("/home/finkensim/finken/paparazzi/sw/tools/ovgu_swarm/WP_Mover.log","a") as LOGGER:
                        LOGGER.write("%d send this valid Goal_Achieved-MSG: %s\n" % (ac_id, recvMsg))
                        LOGGER.write("%d. Epoche...\n" % self.__epoche)
            if (ac_id == self.__START_ID) or change:
                self.__moveWP = []
                for acId in range(self.__START_ID, self.__END_ID):
                    self.__moveWP.append(WP_Mover.__createMSG(self.__ATT_ID,acId,self.__ATT_POINTS[self.__epoche%len(self.__ATT_POINTS)]))
                    self.__moveWP.append(WP_Mover.__createMSG(self.__REP_ID,acId,self.__REP_POINTS[self.__epoche%len(self.__REP_POINTS)]))
                if self.__DEBUG: 
                    with open("/home/finkensim/finken/paparazzi/sw/tools/ovgu_swarm/WP_Mover.log","a") as LOGGER:
                        LOGGER.write("%d. Goal_Achieved-MSG: %s\n" % (self.__counter, recvMsg))
                        self.__counter+=1
                self.__send_msgs()
        

        #log infos about init state
        with open("/home/finkensim/finken/paparazzi/sw/tools/ovgu_swarm/WP_Mover.log","a") as LOGGER:   
            LOGGER.write(str("Start_ID: "+str(self.__START_ID)+"; End_ID: "+str(self.__END_ID)+"; moveWP length: "+str(len(self.__moveWP))+"\n"))
            if self.__DEBUG: 
                for i,msg in enumerate(self.__moveWP): LOGGER.write("%2d.MSG: ac_id = %2d; type = %s\n" % (i,msg["ac_id"],str(type(msg["ac_id"]))))
            LOGGER.write("Init-Epoche...\n")

        #run program
        self.__INTERFACE.start()
        time.sleep(2.5)
        self.__send_msgs()
        self.__bind_id = self.__INTERFACE.subscribe(recv_callback,PprzMessage("telemetry", "GOAL_ACHIEVED"))



#main program
if __name__=='__main__':
    wp_mover:WP_Mover = WP_Mover(30, 40, "flight_plan.cvs")
    wp_mover.run()
    time.sleep(40)
    del wp_mover
    wp_mover = None
    sys.exit()
