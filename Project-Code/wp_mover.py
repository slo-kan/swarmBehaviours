#!/usr/bin/python

#module imports
import sys, time, math
from typing import List, Tuple
from threading import Thread, Lock
from queue import Queue, Empty
from os import path,getenv

#add PPRZ_HOME var to Path
DIR = path.dirname(path.abspath(__file__))
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(DIR, '../../../')))
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

    def __repr__(self)->str:
        return str(self.__coords)
    
    def __str__(self)->str:
        return str(self.__coords)


# Simple but thread safe logger
class ThreadSafe_Logger:
    __FILE_WRITER = None
    __WORKER:Thread = None
    __QUEUE:"Queue[str]" = Queue()
    __finished:bool = False

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
    
    def __del__(self):
        self.close()
        del self




########################
# Function Definitions #
########################

#gets the metric distance between two points given in the LlaCoor_f format
def getDistance(own_pos:Point, goal_pos:Point)->float:
    #GLOBE_RADIUS:int = int(6371000)
    return float( 6371000 * math.acos(                  
        math.sin(own_pos["lat"]) * math.sin(goal_pos["lat"]) +    
        math.cos(own_pos["lat"]) * math.cos(goal_pos["lat"]) *    
        math.cos(own_pos["lon"] - goal_pos["lon"])
    ))


#converts a LLA-Point from float radial representation to a int degree representation
def convertToInt(point:Point)->Tuple[int]:
    lat = int((point["lat"]*180/math.pi)*1e7)
    lon = int((point["lon"]*180/math.pi)*1e7)
    alt = int(point["alt"])
    return (lat,lon,alt)


#converts a LLA-Point from float radial representation to float degree representation
def convertToDegree(point:Point)->Tuple[float]:
    lat = float(point["lat"]*180/math.pi)
    lon = float(point["lon"]*180/math.pi)
    alt = float(point["alt"])
    return (lat,lon,alt)


#create PprzMessage            
def createMSG(wp_id:int,ac_id:int,point:Point)->PprzMessage:
    msg = PprzMessage("ground", "MOVE_WAYPOINT")
    msg['wp_id'] = wp_id
    msg['ac_id'] = str(ac_id)
    msg['lat'],msg['long'],msg['alt'] = convertToDegree(point)
    return msg


#Input Function
#scan file to create flight plan
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



#Main Class Definition
class WPMover:

    #global variables
    __LOGGER:ThreadSafe_Logger = None
    __MSG_SENDING_THREAD:Thread = None
    __MSG_UPDATE_THREAD:Thread = None
    __MSG_ACCESS:Lock = Lock()
    __MSG_QUEUE:"Queue[Tuple[Point,int]]" = Queue()
    __moveWP:List[PprzMessage] = []
    __terminate:bool = False
    __down:bool = False
    __epoche:int = 0
    __counter:int = 1


    #init the WP-Mover tool
    def __init__(self,fname:str,start_id:int,end_id:int,first_att_id:int,att_ids:List[int],rep_ids:List[int],atts:List[Point],reps:List[Point],spawns:List[Point],update_type:str,Debug:bool=False):
        self.__LOG:str = fname
        self.__LOGGER = ThreadSafe_Logger(self.__LOG)
        self.__DEBUG:bool = Debug
        self.__UPDATE:str = update_type
        self.__FIRST_ATT_ID:int = first_att_id
        self.__ATT_IDS:List[int] = att_ids
        self.__REP_IDS:List[int] = rep_ids
        self.__END_ID:int = end_id      #end_id is exclusiv 40 means last copter in the swarm has id 39
        self.__START_ID:int = start_id  #start_id is inclusiv 30 means last copter in the swarm has id 30
        self.__ATT_POINTS:List[Point] = atts
        self.__REP_POINTS:List[Point] = reps
        self.__SPAWN_POINTS:List[Point] = spawns
        self.__INTERFACE = IvyMessagesInterface(
                               agent_name="Pprzlink_Move_WP",      # Ivy agent name
                               start_ivy=False,                    # Do not start the ivy bus now
                               ivy_bus="127.255.255.255:2010"      # address of the ivy bus
                           )

    #initialize terminatation
    def terminate(self):
        self.__terminate = True 
    
    #clean up threads
    def shutdown(self):
        if not self.__down:
            self.terminate()
            self.__MSG_SENDING_THREAD.join()
            self.__MSG_QUEUE.join()
            self.__MSG_UPDATE_THREAD.join()
            self.__LOGGER.close()
            self.__INTERFACE.shutdown()
            self.__down = True
    
    #delete object
    def __del__(self): 
        self.shutdown()
        del self

    
    #main method - executes the program
    def run(self):

        #send update msgs through interface
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

        #recv updates from drones
        def recv_callback(ac_id, recvMsg):
            if(int(recvMsg["achieved"])>0):
                own_pos = Point(float(recvMsg["lat"]),float(recvMsg["lon"]),float(recvMsg["alt"]))
                att_id = int(recvMsg["wp_id"])
                data:Tuple[Point,int,int,PprzMessage] = (own_pos,att_id,ac_id,recvMsg)
                self.__MSG_QUEUE.put(data)
            if self.__DEBUG: 
                self.__LOGGER.write("%d. Goal_Achieved-MSG: %s\n" % (self.__counter, recvMsg))
                self.__counter+=1
        
        #manipulate goal points via msg updates
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

        #run program
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
        # global ćonstants
        #DIR is holds the directory path where this file is placed
        FIRST_ATT_ID:int = 2 #this value influences behaviour across multiple files please be sure to change it everywhere if you change it here
        LOG_NAME:str = DIR+"/WP_Mover.log" #log-file name

        #scan in needed constants from command line arguments and the file 
        FILE_NAME:str = sys.argv[3]
        LAST_SWARM_AC_ID :int = int(sys.argv[2]) #id is exclusiv 40 means last copter in the swarm has id 39
        FIRST_SWARM_AC_ID:int = int(sys.argv[1]) #id is inclusiv 30 means first copter in the swarm has id 30
        ATT_POINTS, REP_POINTS, SPAWN_POINTS, ATT_RANGE, REP_RANGE, UPDATE = get_szenario(FILE_NAME)
        ATT_IDS, REP_IDS = (list(range(FIRST_ATT_ID,ATT_RANGE+FIRST_ATT_ID)),list(range(ATT_RANGE+FIRST_ATT_ID,ATT_RANGE+FIRST_ATT_ID+REP_RANGE)))

        #create log-file
        with open(LOG_NAME,"w") as log: 
            log.write("Start FlightPlan...\n")
            log.write("ATT_IDS: "+str(ATT_IDS)+", REP_IDS: "+str(REP_IDS)+"\n")
            log.write("ATT_POINTS: "+str(len(ATT_POINTS))+", ATTS: "+str(ATT_POINTS)+"\n")
            log.write("REP_POINTS: "+str(len(REP_POINTS))+", REPS: "+str(REP_POINTS)+"\n")
            log.write("SPAWN_POINTS: "+str(len(SPAWN_POINTS))+", SPAWNS: "+str(SPAWN_POINTS)+"\n")
        
        #start program
        time.sleep(3)
        wp_mover:WPMover = WPMover(LOG_NAME,FIRST_SWARM_AC_ID,LAST_SWARM_AC_ID,FIRST_ATT_ID,ATT_IDS,REP_IDS,ATT_POINTS,REP_POINTS,SPAWN_POINTS,UPDATE)
        try: wp_mover.run()
        except: wp_mover.terminate()
        finally: 
            wp_mover.shutdown()
            sys.exit()
