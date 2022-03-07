from cProfile import label
import matplotlib.pyplot as plt
import math

class Point:
  def __init__(self,arr):
    self.__init__(*arr)
  def __init__(self,x,y,z):
    self.x = x
    self.y = y
    self.z = z
    self.coord = [x,y,z]
  def __getitem__(self,idx):
    return self.coord[idx]
  def __sub__(self,other):
    return Point(self.x+other.x,
                 self.y+other.y,
                 self.z+other.z)
  def mag(self):
    return math.sqrt(self.x*self.x+
                     self.y*self.y+
                     self.z*self.z)
  def scale_up(self,s):
    return Point(self.x*s,
                 self.y*s,
                 self.z*s)
    
COMPLEX = False
GRAVITY = 1.985

''' Goal Func Param '''
GOAL_MULT  = 1.75  # g_mult parameter
GOAL_LIMIT = 30.0  # g_lim parameter
GOAL_SIGMA = 4.0   # g_sig parameter  #-3
GOAL_GAMMA = 25.0  # g_gam parameter

''' Danger Func Param '''
DANGER_MULT     = 1.75  # d_mult parameter
DANGER_TO_CLOSE = 8.0   # d_tc parameter
DANGER_LIMIT    = 40.0  # d_lim parameter
DANGER_CUT_OFF  = 5.0   # d_co parameter
DANGER_SIGMA    = 3.85  # d_sig parameter
DANGER_GAMMA    = 18.5  # d_gam parameter
DANGER_ALPHA    = 20.0  # d_alf parameter

''' Drone Attraction Func Param '''
DRONE_ATT_MULT    = 4.0   # da_mult parameter
DRONE_ATT_LIMIT   = 40.0  # da_lim parameter
DRONE_ATT_CUT_OFF = 20.0  # da_co parameter    #18.9
SWARM_DIST        = 12.0  # da_sd parameter

''' Drone Repulsion Func Param '''
DRONE_TO_CLOSE  = 4.0   # dr_dtc parameter
DRONE_REP_MULT  = 4.0   # dr_mult parameter
DRONE_REP_LIMIT = 30.0  # dr_lim parameter
DRONE_REP_SIGMA = 3.0   # dr_sig parameter     #10
DRONE_REP_GAMMA = 1.5   # dr_gam parameter
DRONE_REP_ALPHA = 0.1   # dr_alf parameter


#calculate repulsion forces to other drones
def linear_Repulsion(pos:Point, target:Point, limit:float,
                     sigma:float, gamma:float, alpha:float)->Point:
  force = target-pos
  magnitude = force.mag()
  d = min(magnitude, limit)

  strength = max((sigma/(d+alpha)-gamma),0)*(-1) if COMPLEX else (-1)*sigma*GRAVITY/max(1,d)

  force = force.scale_up(strength/max(magnitude,1))
  return force

#calculate attraction forces to other drones
def log_Attraction(pos:Point, target:Point, limit:float, cutOff:float)->Point:
  force = target - pos
  magnitude = force.mag()
  d = min(magnitude, limit)

  # strength = GRAVITY*d-10  #simple linear
  if COMPLEX:
    strength = math.log(d-(cutOff-GRAVITY/2))*GRAVITY if(d>cutOff)else 0
  else:
    strength = math.log(1+(d-cutOff))*GRAVITY if(d>cutOff)else (GRAVITY/d if(d>cutOff/2)else 0)

  force = force.scale_up(strength/max(magnitude,1))
  return force

# calculate attraction forces with GOAL_VECTORS
def linear_Attraction(pos:Point, target:Point, limit:float,
                      sigma:float, gamma:float=0)->Point:
  force = target - pos
  magnitude = force.mag()
  d = min(magnitude, limit)

  strength = (GRAVITY/sigma)*d+gamma if COMPLEX else (GRAVITY*sigma)/max(d,1)
  
  force = force.scale_up(strength/max(magnitude,1))
  return force

# calculate repulsion force with DANGER_VECTORS
def limExp_Repulsion(pos:Point, target:Point, limit:float, cutOff:float, 
                     sigma:float, gamma:float, alpha:float)-> Point:
  force = target-pos
  magnitude = force.mag()
  d = min(magnitude, limit)
  
  if COMPLEX:
    strength = GRAVITY*(d/2-limit)/sigma if(d<cutOff)else (-1)*math.exp((-1)*(GRAVITY*(alpha*math.log(d)-gamma)/sigma))
  else:
    strength = (-1)*(GRAVITY*sigma)/max(d,1) if(d<cutOff)else (-1)*math.exp((-1)*GRAVITY*math.log(d))

  force = force.scale_up(strength/max(magnitude,1))
  return force


test_min = -60
test_max = 60
pos = Point(0,0,0)
targets = [Point(it,0,0) for it in range(test_min,test_max)]
goal_vals,danger_vals,d_att_vals,d_rep_vals,dists = ([],[],[],[],[])

for target in targets:
  goal_vals.append(linear_Attraction(pos,target,GOAL_LIMIT,GOAL_SIGMA,GOAL_GAMMA).mag())
  d_att_vals.append(log_Attraction(pos,target,DRONE_ATT_LIMIT,DRONE_ATT_CUT_OFF).mag())
  negative = False
  for coord in linear_Repulsion(pos,target,DRONE_REP_LIMIT,DRONE_REP_SIGMA,DRONE_REP_GAMMA,DRONE_REP_ALPHA): 
    if coord<0: 
      negative = True
      break
  d_rep_vals.append(linear_Repulsion(pos,target,DRONE_REP_LIMIT,DRONE_REP_SIGMA,DRONE_REP_GAMMA,DRONE_REP_ALPHA).mag()*(-1 if negative else 1))
  negative = False
  for coord in limExp_Repulsion(pos,target,DANGER_LIMIT,DANGER_CUT_OFF,DANGER_SIGMA,DANGER_GAMMA,DANGER_ALPHA): 
    if coord<0: 
      negative = True
      break
  danger_vals.append(limExp_Repulsion(pos,target,DANGER_LIMIT,DANGER_CUT_OFF,DANGER_SIGMA,DANGER_GAMMA,DANGER_ALPHA).mag()*(-1 if negative else 1))
  negative = False
  for coord in (target-pos): 
    if coord<0: 
      negative = True
      break
  dists.append((target-pos).mag()*(-1 if negative else 1))

fig = plt.figure()
plt.plot(dists,goal_vals,color='green',label="Goal-Magnitude")
plt.plot(dists,danger_vals,color='red',label="Danger-Magnitude")
plt.plot(dists,d_att_vals,color='cyan',label="Drone_ATT-Magnitude")
plt.plot(dists,d_rep_vals,color='orange',label="Drone_REP-Magnitude")
plt.legend()
fig.suptitle("Scoring Function Results")

fig,axes = plt.subplots(2,2)
axes[0,0].plot(dists[test_min:test_min+int(math.ceil(GOAL_LIMIT))],goal_vals[test_min:test_min+int(math.ceil(GOAL_LIMIT))],color='green',label="Goal-Magnitude")
axes[0,1].plot(dists[test_min:test_min+int(math.ceil(DANGER_LIMIT))],danger_vals[test_min:test_min+int(math.ceil(DANGER_LIMIT))],color='red',label="Danger-Magnitude")
axes[1,0].plot(dists[test_min:test_min+int(math.ceil(DRONE_ATT_LIMIT))],d_att_vals[test_min:test_min+int(math.ceil(DRONE_ATT_LIMIT))],color='cyan',label="Drone_ATT-Magnitude")
axes[1,1].plot(dists[test_min:test_min+int(math.ceil(DRONE_REP_LIMIT))],d_rep_vals[test_min:test_min+int(math.ceil(DRONE_REP_LIMIT))],color='orange',label="Drone_REP-Magnitude")
for arr in axes: 
  for ax in arr: 
    ax.legend()
fig.suptitle("Limited Scoring Function Results")

plt.show()