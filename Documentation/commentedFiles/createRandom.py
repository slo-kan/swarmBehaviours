#!/usr/bin/python

## @file createRandom.py
#  @brief An algorithm to create random scenario with an seed

## @brief module imports
import sys,random
 
## @brief calculates random spawnpoint within spwan_dist distance to swarm center
#  @return (x,y) tupel with created coordinates
def getSpawnPoint(swarm_center:"tuple[float]", spawn_dist:int, scale:"tuple[float]")->"tuple[float]":
    x = random.gauss(swarm_center[0],spawn_dist)/100000+scale[0]
    y = random.gauss(swarm_center[1],spawn_dist)/100000+scale[1]
    return (x,y)

## @brief choose the starting and ending point of a 250m diagonal that has your point of intrest in its middle
#  @details FIN - (52.1389, 11.64583) -> Start: (52.13736, 11.64373), End: (52.13974, 11.64875)
def main(argv:"list[str]"):
    START_X = float(argv[1])
    START_Y = float(argv[2])
    END_X   = float(argv[3])
    END_Y   = float(argv[4])
    SPAWNS  = int(argv[5])
    SEED = int(argv[6]) if len(argv)==7 else random.randint(0,9999999)
    random.seed(SEED)
    print(SEED)

    ATTS = random.randint(1,15)
    REPS = random.randint(1,15)
    spawn_dist = random.randint(10,75) 
    swarm_center = (random.gauss((START_X+END_X)/2,68),random.gauss((START_Y+END_Y)/2,68))
    spawn_points = [getSpawnPoint(swarm_center,spawn_dist,(START_X,START_Y)) for _ in range(SPAWNS)]
    att_points = [(random.uniform(START_X,END_X),random.uniform(START_Y,END_Y)) for _ in range(ATTS*6)]
    rep_points = [(random.uniform(START_X,END_X),random.uniform(START_Y,END_Y)) for _ in range(REPS*6)]
    with open(("random_"+str(SEED)+".csv"),"w") as file:
        file.write("update:single_rep, att:"+str(ATTS)+", rep:"+str(REPS)+"\n")
        file.write("lat,                lon,                type\n")
        for point in rep_points: file.write("%.15f, %.15f, rep\n"%(point[0],point[1]))
        for point in att_points: file.write("%.15f, %.15f, att\n"%(point[0],point[1]))
        for point in spawn_points: file.write("%.15f, %.15f, spawn\n"%(point[0],point[1]))
    with open(("random_"+str(SEED)+"_swarm_positions.csv"),"w") as file:
        file.write("att:"+str(ATTS)+", rep:"+str(REPS)+", spawn:"+str(SPAWNS)+"\n")
        file.write("lat,                lon\n")
        for point in spawn_points: file.write("%.15f, %.15f\n"%(point[0],point[1]))

## @brief main program
if __name__=='__main__':
    if len(sys.argv)>5: main(sys.argv)