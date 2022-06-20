#!/usr/bin/python

#module imports
import sys, time, math
from typing import List, Tuple
from threading import Thread, Lock
from queue import Queue, Empty
from os import path,getenv


## Holds the directory path where the file is placed.
DIR = path.dirname(path.abspath(__file__))
## Environment variable of the path
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(DIR, '../../../')))
sys.path.append(PPRZ_HOME+ "/var/lib/python")

## Import pprzlink interface components
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage


#####################
# Class Definitions #
#####################

## Defination of a class, Point.
#  @details Point is a tuple of coordinates containing latitude, longitude, and altitude in the same order and
#  can be accesed by {lat,lon,alt}, {x,y,z} or {0,1,2}. The altitude of all the aircrafts is fixed to 47 meters
#  to work with only two dimensions(latitude and longitude).
class Point:
    @property
    def __IDXS(self): return {0: 0, 1: 1, 2: 2, 'lat': 0, 'lon': 1, 'alt': 2, 'x': 0, 'y': 1, 'z': 2}
    ## @var __STANDARD_ALT
    #  Altitude for all the points further used
    __STANDARD_ALT:float = 47.0

    def __init__(self,x:float,y:float,z:float=__STANDARD_ALT):
        self.__coords = [x,y,z]

    def __getitem__(self,idx)->float:
        return float(self.__coords[self.__IDXS[idx]])
    
    def __setitem__(self,idx,val:float):
        self.__coords[self.__IDXS[idx]] = val

    def __repr__(self)->str:
        return str(self.__coords)
    
    def __str__(self)->str:
        return str(self.__coords)


## Deination of a class, ThreadSafe_Logger.
#  @detais Simple but thread safe logger.
class ThreadSafe_Logger:
    ## @var __FILE_WRITER
    #  stores the file writter to log all the messages
    __FILE_WRITER = None
    ## @var __WORKER
    #  stores the thread used for logging
    __WORKER:Thread = None
    ## @var __QUEUE
    #  stores a queue of srings to store messages
    __QUEUE:"Queue[str]" = Queue()
    ## @var __finished
    #  stores a boolean which is true after the logging thread is closed
    __finished:bool = False

    ## @brief writes the data in queue to the file
    def __write_to_file(self):
        while not self.__FILE_WRITER: time.sleep(1)
        counter = 0
        while not self.__finished or not self.__QUEUE.empty():
            try: data = self.__QUEUE.get(timeout=0.1)
            except Empty: time.sleep(0.5)
            else: 
                self.__FILE_WRITER.write(data)
                self.__QUEUE.task_done()
                if counter==10: 
                    self.__FILE_WRITER.flush()
                    counter = 0
                else: counter+=1

    ## The constructor
    #  @param filename name of the file to write to.
    def __init__(self, filename):
        #  @var __FILENAME
        #  name of the file to write to
        self.__FILENAME = filename
        self.__WORKER = Thread(target=self.__write_to_file,name="Logging_Thread")
    
    ## Starts the instance, log file is opened and the thread is started.
    #  @param self The object pointer.
    def start(self):
        self.__FILE_WRITER = open(self.__FILENAME,"a")
        self.__WORKER.start()
        self.write("Logging_Thread started...\n")

    ## Adds the data to the queue.
    #  @param self The object pointer.
    #  @param data the message data.
    def write(self, data:str): self.__QUEUE.put(data)

    ## Closes the instance, blocks the queue and thread untill all the items of queue are processed.
    #  @param self The object pointer.
    def close(self):
        self.write("... Logging_Thread closed!\n")
        self.__finished = True
        self.__QUEUE.join()
        self.__WORKER.join()
        self.__FILE_WRITER.close() 
    ## Calls the method close.
    #  @param self The object pointer.
    def __del__(self):
        self.close()
        del self




########################
# Function Definitions #
########################

## Gets the metric distance between two points given in the LlaCoor_f format.
#  @param own_pos Own position.
#  @param goal_pos goal position.
#  @details the metric distance is calculated by calculating the length of an arc made by the two points on the globe which equals to
#  radius of the Earth multiplied by the angle made by the arc at the center of the globe. And the angle made by arc of points at
#  the center of globe is calculated using the latitudes and longitudes of the points.
#  GLOBE_RADIUS = 6371000 meters
#  @return metric distance(meters) in float.

def getDistance(own_pos:Point, goal_pos:Point)->float:
    #GLOBE_RADIUS:int = int(6371000)
    return float( 6371000 * math.acos(                  
        math.sin(own_pos["lat"]) * math.sin(goal_pos["lat"]) +    
        math.cos(own_pos["lat"]) * math.cos(goal_pos["lat"]) *    
        math.cos(own_pos["lon"] - goal_pos["lon"])
    ))


## Converts a LLA-Point from float radial representation to a int degree representation.
#  @param point A point as a tuple of floats (lat,lon,alt), representing angles in radians.
#  @details the radial representation of an LLA_Point which is in radians is converted to degrees
#  by using,PI radians equals 180 degrees and to convert the obtained float value to integer,
#  is multiplied by 1e7 is to take into account 7 digits after decimal point of the float for the integer values to be significant.
#  @return a tuple of integers (lat,lo,alt).

def convertToInt(point:Point)->Tuple[int]:
    lat = int((point["lat"]*180/math.pi)*1e7)
    lon = int((point["lon"]*180/math.pi)*1e7)
    alt = int(point["alt"])
    return (lat,lon,alt)


## Converts a LLA-Point from float radial representation to float degree representation.
#  @param point A point as a tuple of floats (lat,lon,alt), representing angles in radians.
#  @details the radial representation of an LLA_Point which is in radians is converted to degrees
#  by using,PI radians equals 180 degrees.
#  @return a tuple of floats (lat,lo,alt).

def convertToDegree(point:Point)->Tuple[float]:
    lat = float(point["lat"]*180/math.pi)
    lon = float(point["lon"]*180/math.pi)
    alt = float(point["alt"])
    return (lat,lon,alt)


## Creates PprzMessage.
#  @param wp_id Waypoint id.
#  @param ac_id Aircraft id.
#  @param point Coordinates of a point in radial representation.
#  @details creats a paparazzi message consisting of way point id, air craft id and coordinates of a point in degrees.
#  @return paparazzi message.

def createMSG(wp_id:int,ac_id:int,point:Point)->PprzMessage:
    msg = PprzMessage("ground", "MOVE_WAYPOINT")
    msg['wp_id'] = wp_id
    msg['ac_id'] = str(ac_id)
    msg['lat'],msg['long'],msg['alt'] = convertToDegree(point)
    return msg


## Input Function.
#  @param filename Name of the file to read from.
#  @details scan file to create flight plan and get required data for flight.
#  @return a list of attraction points, a list pf repel points, a list of spawn points, no of attraction points,
#  number of repel points and type of update.

def get_szenario(filename)->Tuple[List[Point]]:
    att_points,rep_points,spawn_points = ([],[],[])
    atts, reps = (0,0)
    file = open((DIR+"/"+filename),"r")
    lines = file.readlines()
    values = lines[0].strip().split(",")
    for val in values: 
        if val.strip().split(":")[0]=="att":
            atts = int(val.strip().split(":")[-1])
        elif val.strip().split(":")[0]=="rep":
            reps = int(val.strip().split(":")[-1])
        elif val.strip().split(":")[0]=="update":
            update = val.strip().split(":")[-1].strip()
    for line in lines[2:]:
        data = line.strip().split(",")
        if data[-1].strip() == "att":
            att_points.append(Point((float(data[0])*math.pi/180),(float(data[1])*math.pi/180)))
        elif data[-1].strip() == "rep":
            rep_points.append(Point((float(data[0])*math.pi/180),(float(data[1])*math.pi/180)))
        elif data[-1].strip() == "spawn":
            spawn_points.append(Point((float(data[0])*math.pi/180),(float(data[1])*math.pi/180)))
    return (att_points,rep_points,spawn_points,atts,reps,update)



## Main Class Definition
class WPMover:

    ## @var __LOGGER
    #  is a thread safe logger
    __LOGGER:ThreadSafe_Logger = None
    ## @var __MSG_SENDING_THREAD
    #  is a thread for sending messgaes
    __MSG_SENDING_THREAD:Thread = None
    ## @var __MSG_UPDATE_THREAD
    #  is a thread for updating messages
    __MSG_UPDATE_THREAD:Thread = None
    ## @var __MSG_ACCESS
    #  is a lock for synchronization to block two threads from trying to read/write the same shared object
    __MSG_ACCESS:Lock = Lock()
    ## @var __MSG_QUEUE
    #  is a queue to store messages
    __MSG_QUEUE:"Queue[Tuple[Point,int]]" = Queue()
     ## @var __moveWP
    #  is a list of messages
    __moveWP:List[PprzMessage] = []
    ## @var __terminate
    #  is a boolean which if true does not run the method
    __terminate:bool = False
    ## @var __down
    #  is a boolean used to clean up threds
    __down:bool = False
    ## @var __epoche
    #  is an integer used to uptade the waypoints
    __epoche:int = 0
    ## @var __counter
    #  is an integer used in dubug
    __counter:int = 1


      ## The Constructor.
    #  @param fname logfile name
    #  @param start_id first copter id
    #  @param end_id last copter id + 1
    #  @param first_att_id first attracion id
    #  @param att_ids list of attraction ids
    #  @param rep_ids list of repel ids
    #  @param atts list of attraction points
    #  @param reps list of repel points
    #  @param spawns list of spawn points
    #  @param update_type type of update
    #  @param Debug boolean to debug or not
    def __init__(self,fname:str,start_id:int,end_id:int,first_att_id:int,att_ids:List[int],rep_ids:List[int],atts:List[Point],reps:List[Point],spawns:List[Point],update_type:str,Debug:bool=False):
        ## @var __LOG
        #  holds the name of logfile
        self.__LOG:str = fname
        ## @var __LOGGER
        #  writes to log file
        self.__LOGGER = ThreadSafe_Logger(self.__LOG)
        ## @var __DEBUG
        #  is a boolean whether to debug or not
        self.__DEBUG:bool = Debug
        ## @var __UPDATE
        #  is the type of update
        self.__UPDATE:str = update_type
        ## @var __FIRST_ATT_ID
        #  is the first attraction id
        self.__FIRST_ATT_ID:int = first_att_id
         ## @var __ATT_IDS
        #  is the list of attraction ids
        self.__ATT_IDS:List[int] = att_ids
        ## @var __REP_IDS
        #  is the list of repel ids
        self.__REP_IDS:List[int] = rep_ids
        ## @var __END_ID
        #  is exclusiv 40 means last copter in the swarm has id 39
        self.__END_ID:int = end_id
        ## @var __START_ID
        #  is inclusiv 30 means last copter in the swarm has id 30
        self.__START_ID:int = start_id
        ## @var __ATT_POINTS
        #  is the list of attraction points
        self.__ATT_POINTS:List[Point] = atts
        ## @var __REP_POINTS
        #  is the list of repel points
        self.__REP_POINTS:List[Point] = reps
        ## @var __SPAWN_POINTS
        #  is the list of spawn points
        self.__SPAWN_POINTS:List[Point] = spawns
        ## @var INTERFACE
        #  is the message interface
        self.__INTERFACE = IvyMessagesInterface(
                               ## agent_name is Ivy agent name
                               agent_name="Pprzlink_Move_WP",
                               ## start_ivy is a a bolean which if false does not start the ivy bus now
                               start_ivy=False,
                               ## ivy_bus is the address of the ivy bus
                               ivy_bus="127.255.255.255:2010"
                           )
    ## Initializes terminatation.
    #  @param self The object pointer.
    def terminate(self):
        self.__terminate = True 
    
    ## Cleans threads.
    #  @param self The object pointer.
    def shutdown(self):
        if not self.__down:
            self.terminate()
            self.__MSG_SENDING_THREAD.join()
            self.__MSG_QUEUE.join()
            self.__MSG_UPDATE_THREAD.join()
            self.__LOGGER.close()
            self.__INTERFACE.shutdown()
            self.__down = True
    
    ## Delete object.
    #  @param self The object pointer.
    def __del__(self): 
        self.shutdown()
        del self

    
     ## Main method - executes the program.
    #  @param self The object pointer.
    def run(self):

        ## Sends update msgs through interface and write it to the log file.
        def send_msgs():
            self.__LOGGER.write("MSG-Thread started ... \n")
            while not self.__terminate:
                aquired = self.__MSG_ACCESS.acquire(timeout=0.75)
                if aquired:
                    if self.__DEBUG: self.__LOGGER.write("len(moveWP) = %d\n" % len(self.__moveWP))
                    string = ""
                    for i,msg in enumerate(self.__moveWP):
                        time.sleep(0.05)
                        self.__INTERFACE.send(msg)
                        pre = str("ATT" if msg['wp_id'] in self.__ATT_IDS else "REP")
                        string += ("%2d. %3s-WP_Move-MSG: %s\n" % (i, pre, msg))
                    if self.__DEBUG: self.__LOGGER.write(string)
                    self.__MSG_ACCESS.release()
                    time.sleep(3)
            self.__LOGGER.write("... MSG-Thread closed! \n")

        ## Recursive updates from drones
        #  @param ac_is Aircraft id.
        #  @param recvMsg Is the teemetry message consisting of waypoint id, its coordiantes.
        #  Can be found in messages.xml with the message name "GOAL_ACHIEVED" aad id "76".
        def recv_callback(ac_id, recvMsg):
            if(int(recvMsg["achieved"])>0):
                own_pos = Point(float(recvMsg["lat"]),float(recvMsg["lon"]),float(recvMsg["alt"]))
                att_id = int(recvMsg["wp_id"])
                data:Tuple[Point,int,int,PprzMessage] = (own_pos,att_id,ac_id,recvMsg)
                self.__MSG_QUEUE.put(data)
            if self.__DEBUG: 
                self.__LOGGER.write("%d. Goal_Achieved-MSG: %s\n" % (self.__counter, recvMsg))
                self.__counter+=1
        
        ## Manipulates/changes waypoints via msg updates.
        #  @details the diatance between the aircraft and waypoint is calculated and if it is less than 16.25,
        #  the waypoint is updated according to the type of update. Here are three types of updates,
        #  all : if one aircrafts comes closer than the required distance waypoints for all the drones are updated.
        #  single : only the waypoints of the aircrafts which came closer are updated.
        #  delete : waypoins are updated to spawn points.
        def update_waypoint():
            while not self.__terminate or not self.__MSG_QUEUE.empty():
                try: data = self.__MSG_QUEUE.get(timeout=0.1)
                except Empty: time.sleep(0.5)
                else: 
                    own_pos,att_id,ac_id,recvMsg = data
                    if(getDistance(own_pos,self.__ATT_POINTS[((att_id-2)+len(self.__ATT_IDS)*self.__epoche)%len(self.__ATT_POINTS)])<=16.25):
                        with self.__MSG_ACCESS:
                            self.__epoche += 1
                            self.__moveWP = []
                            for acId in range(self.__START_ID, self.__END_ID):
                                if "all" in self.__UPDATE:
                                    for idx,attID in enumerate(self.__ATT_IDS): 
                                        self.__moveWP.append(createMSG(attID,acId,self.__ATT_POINTS[(idx+len(self.__ATT_IDS)*self.__epoche)%len(self.__ATT_POINTS)]))
                                    if self.__UPDATE == "all_rep":
                                        for idx,repID in enumerate(self.__REP_IDS): 
                                            self.__moveWP.append(createMSG(repID,acId,self.__REP_POINTS[(idx+len(self.__REP_IDS)*self.__epoche)%len(self.__REP_POINTS)]))
                                else: 
                                    idx = att_id-self.__FIRST_ATT_ID
                                    if "single" in self.__UPDATE:
                                        self.__moveWP.append(createMSG(att_id,acId,self.__ATT_POINTS[(idx+len(self.__ATT_IDS)*self.__epoche)%len(self.__ATT_POINTS)]))
                                        if self.__UPDATE == "single_rep" and idx in range(len(self.__REP_IDS)):
                                            rep_id = self.__REP_IDS[idx]
                                            self.__moveWP.append(createMSG(rep_id,acId,self.__REP_POINTS[(idx+len(self.__REP_IDS)*self.__epoche)%len(self.__REP_POINTS)]))
                                    elif "delete" in self.__UPDATE:
                                        self.__moveWP.append(createMSG(att_id,acId,self.__SPAWN_POINTS[(acId-self.__START_ID)]))
                                        if self.__UPDATE == "delete_rep":
                                            if idx in range(len(self.__REP_IDS)):
                                                rep_id = self.__REP_IDS[idx]
                                                self.__moveWP.append(createMSG(rep_id,acId,self.__SPAWN_POINTS[(acId-self.__START_ID)]))
                                    elif self.__DEBUG: self.__LOGGER.write("Update: %s\n"%self.__UPDATE)
                            self.__LOGGER.write("%d send this valid Goal_Achieved-MSG: %s\n" % (ac_id, recvMsg))
                            self.__LOGGER.write("%d. Epoche\n" % self.__epoche)
                    elif self.__DEBUG: self.__LOGGER.write("ATT_ID: %d\n"%att_id)
                    self.__MSG_QUEUE.task_done()

        #init vars
        self.__MSG_SENDING_THREAD = Thread(target=send_msgs,name="Send_Msg_Thread")
        self.__MSG_UPDATE_THREAD = Thread(target=update_waypoint,name="Update_Msg_Thread")
        self.__LOGGER.start()
        with self.__MSG_ACCESS:
            for acId in range(self.__START_ID, self.__END_ID):
                for idx,attID in enumerate(self.__ATT_IDS): self.__moveWP.append(createMSG(attID,acId,self.__ATT_POINTS[idx]))
                for idx,repID in enumerate(self.__REP_IDS): self.__moveWP.append(createMSG(repID,acId,self.__REP_POINTS[idx]))    
        self.__LOGGER.write(str("Start_ID: "+str(self.__START_ID)+"; End_ID: "+str(self.__END_ID)+"; moveWP length: "+str(len(self.__moveWP))+"\n"))
        self.__LOGGER.write("Init-Epoche\n")

        ## Interface and threads are started and if any exception raises STOPPING FLIGHTPLAN is written to the log file.
        try:
            self.__INTERFACE.start()
            self.__MSG_SENDING_THREAD.start()
            self.__INTERFACE.subscribe(recv_callback,PprzMessage("telemetry", "GOAL_ACHIEVED"))
            self.__MSG_UPDATE_THREAD.start()
            while not self.__terminate: time.sleep(15)
            raise KeyboardInterrupt 
        except: self.__LOGGER.write("... !STOPPING FlightPlan! ...\n")
        finally:
            self.shutdown() 
            raise KeyboardInterrupt

        


#main program
if __name__=='__main__': 
    if len(sys.argv)==4:

        ## FIRST_ATT_ID defines the first attraction id, also influences behaviour across multiple files.
        FIRST_ATT_ID:int = 2
        ## Holds the log-file name.
        LOG_NAME:str = DIR+"/WP_Mover.log"

        ## FILE_NAME scans in needed constants from command line arguments and the file.
        FILE_NAME:str = sys.argv[3]
        ## LAST_SWARM_AC_ID is exclusiv 40 means last copter in the swarm has id 39.
        LAST_SWARM_AC_ID :int = int(sys.argv[2])
        ## FIRST_SWARM_AC_ID is inclusiv 30 means first copter in the swarm has id 30.
        FIRST_SWARM_AC_ID:int = int(sys.argv[1])
        ATT_POINTS, REP_POINTS, SPAWN_POINTS, ATT_RANGE, REP_RANGE, UPDATE = get_szenario(FILE_NAME)
        ATT_IDS, REP_IDS = (list(range(FIRST_ATT_ID,ATT_RANGE+FIRST_ATT_ID)),list(range(ATT_RANGE+FIRST_ATT_ID,ATT_RANGE+FIRST_ATT_ID+REP_RANGE)))

        ## Creates log-file and writes to it the flightplan data
        with open(LOG_NAME,"w") as log: 
            log.write("Start FlightPlan...\n")
            log.write("ATT_IDS: "+str(ATT_IDS)+", REP_IDS: "+str(REP_IDS)+"\n")
            log.write("ATT_POINTS: "+str(len(ATT_POINTS))+", ATTS: "+str(ATT_POINTS)+"\n")
            log.write("REP_POINTS: "+str(len(REP_POINTS))+", REPS: "+str(REP_POINTS)+"\n")
            log.write("SPAWN_POINTS: "+str(len(SPAWN_POINTS))+", SPAWNS: "+str(SPAWN_POINTS)+"\n")
        
        ##start program
        time.sleep(3)
        wp_mover:WPMover = WPMover(LOG_NAME,FIRST_SWARM_AC_ID,LAST_SWARM_AC_ID,FIRST_ATT_ID,ATT_IDS,REP_IDS,ATT_POINTS,REP_POINTS,SPAWN_POINTS,UPDATE)
        try: wp_mover.run()
        except: wp_mover.terminate()
        finally: 
            wp_mover.shutdown()
            sys.exit()