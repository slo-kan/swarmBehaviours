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
    
    def __del__(self):
        self.close()



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
GLOBE_RADIUS:int = int(6371000)



########################
# Function Definitions #
########################

#gets the metric distance between two points given in the LlaCoor_f format
def getDistance(own_pos:Point, goal_pos:Point)->float:
    return float( GLOBE_RADIUS * math.acos(                  
        math.sin(own_pos["lat"]) * math.sin(goal_pos["lat"]) +    
        math.cos(own_pos["lat"]) * math.cos(goal_pos["lat"]) *    
        math.cos(own_pos["lon"] - goal_pos["lon"])
    ))


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
    msg['lat'],msg['long'],msg['alt'] = convertToDegree(point)
    return msg



class WPMover:

    #global variables
    __MSG_SENDING_THREAD:Thread = None
    __MSG_UPDATE_THREAD:Thread = None
    __MSG_ACCESS:Lock = Lock()
    __MSG_QUEUE:"Queue[Point]" = Queue()
    __moveWP:"list[PprzMessage]" = []
    __terminate:bool = False
    __epoche:int = 0
    __counter:int = 1


    #init the WP-Mover tool
    def __init__(self,Logger:ThreadSafe_Logger,start_id:int,end_id:int,att_id:int,rep_id:int,atts:"list[Point]",reps:"list[Point]",Debug:bool=False):
        self.__LOGGER:ThreadSafe_Logger = Logger
        self.__DEBUG:bool = Debug
        self.__ATT_ID:int = att_id
        self.__REP_ID:int = rep_id
        self.__END_ID:int = end_id      #end_id is exclusiv 40 means last copter in the swarm has id 39
        self.__START_ID:int = start_id  #start_id is inclusiv 30 means last copter in the swarm has id 30
        self.__ATT_POINTS:"list[Point]" = atts
        self.__REP_POINTS:"list[Point]" = reps
        self.__INTERFACE = IvyMessagesInterface(
                agent_name="Pprzlink_Move_WP",      # Ivy agent name
                start_ivy=False,                    # Do not start the ivy bus now
                ivy_bus="127.255.255.255:2010"      # address of the ivy bus
            )

    #clean up threads
    def shutdown(self):
        self.__terminate = True
        self.__MSG_SENDING_THREAD.join()
        self.__MSG_QUEUE.join()
        self.__MSG_UPDATE_THREAD.join()
        time.sleep(3)
        self.__LOGGER.close()
        self.__INTERFACE.shutdown()
    
    #delete object
    def __del__(self): self.close()

    
    #main method - executes the program
    def run(self):

        #send update msgs through interface
        def send_msgs():
            self.__LOGGER.write("MSG-Thread started ... \n")
            while not self.__terminate:
                aquired = self.__MSG_ACCESS.acquire(timeout=0.5)
                if aquired:
                    if self.__DEBUG: self.__LOGGER.write("len(moveWP) = %d\n" % len(self.__moveWP))
                    string = ""
                    for i,msg in enumerate(self.__moveWP):
                        time.sleep(0.05)
                        self.__INTERFACE.send(msg)
                        pre = str("ATT" if self.__ATT_ID == msg['wp_id'] else "REP")
                        string += ("%2d. %3s-WP_Move-MSG: %s\n" % (i, pre, msg))
                    if self.__DEBUG: self.__LOGGER.write(string)
                    self.__MSG_ACCESS.release()
                    time.sleep(5)
            self.__LOGGER.write("... MSG-Thread closed! \n")

        #recv updates from drones
        def recv_callback(ac_id, recvMsg):
            if(int(recvMsg["achieved"])>0):
                own_pos = Point(float(recvMsg["lat"]),float(recvMsg["lon"]),float(recvMsg["alt"]))
                self.__MSG_QUEUE.put(own_pos)
                self.__LOGGER.write("%d send this valid Goal_Achieved-MSG: %s\n" % (ac_id, recvMsg))
            if self.__DEBUG: 
                self.__LOGGER.write("%d. Goal_Achieved-MSG: %s\n" % (self.__counter, recvMsg))
                self.__counter+=1
        
        #manipulate goal points via msg updates
        def update_waypoint():
            while not self.__terminate or not self.__MSG_QUEUE.empty():
                try: own_pos:Point = self.__MSG_QUEUE.get(timeout=0.2)
                except Empty: time.sleep(1)
                else: 
                    if(getDistance(own_pos,self.__ATT_POINTS[self.__epoche%len(self.__ATT_POINTS)])<=16.25):
                        with self.__MSG_ACCESS:
                            self.__epoche += 1
                            self.__moveWP = []
                            for acId in range(self.__START_ID, self.__END_ID):
                                self.__moveWP.append(createMSG(self.__ATT_ID,acId,self.__ATT_POINTS[self.__epoche%len(self.__ATT_POINTS)]))
                                self.__moveWP.append(createMSG(self.__REP_ID,acId,self.__REP_POINTS[self.__epoche%len(self.__REP_POINTS)]))
                        self.__LOGGER.write("%d. Epoche\n" % self.__epoche)
                    self.__MSG_QUEUE.task_done()

        #init vars
        self.__MSG_SENDING_THREAD = Thread(target=send_msgs,name="Send_Msg_Thread")
        self.__MSG_UPDATE_THREAD = Thread(target=update_waypoint,name="Update_Msg_Thread")
        self.__LOGGER.start()
        with self.__MSG_ACCESS:
            for acId in range(self.__START_ID, self.__END_ID):
                self.__moveWP.append(createMSG(self.__ATT_ID,acId,self.__ATT_POINTS[0]))
                self.__moveWP.append(createMSG(self.__REP_ID,acId,self.__REP_POINTS[0]))    
        self.__LOGGER.write(str("Start_ID: "+str(self.__START_ID)+"; End_ID: "+str(self.__END_ID)+"; moveWP length: "+str(len(self.__moveWP))+"\n"))
        self.__LOGGER.write("Init-Epoche\n")

        #run program
        try:
            self.__INTERFACE.start()
            self.__MSG_SENDING_THREAD.start()
            self.__INTERFACE.subscribe(recv_callback,PprzMessage("telemetry", "GOAL_ACHIEVED"))
            self.__MSG_UPDATE_THREAD.start()
            while True: time.sleep(15)
        except: self.__LOGGER.write("... !STOPPING FlightPlan! ...\n")
        finally: self.shutdown()

        


#main program
if __name__=='__main__':
    ATT_POINTS, REP_POINTS = get_szenario("flight_plan.cvs")
    out = open("/home/finkensim/finken/paparazzi/sw/tools/ovgu_swarm/WP_Mover.log","w")
    out.write("Start FlightPlan...\n")
    out.close()
    time.sleep(3)
    LOGGER:ThreadSafe_Logger = ThreadSafe_Logger("/home/finkensim/finken/paparazzi/sw/tools/ovgu_swarm/WP_Mover.log")
    wpmover:WPMover = WPMover(LOGGER,30,40,2,3,ATT_POINTS,REP_POINTS)
    wpmover.run()
    try: 
        while True: time.sleep(15)
    except: True == True
    finally:
        del wpmover
        del LOGGER
        wpmover = None
        LOGGER = None
        sys.exit()