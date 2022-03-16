from cProfile import label
import matplotlib.pyplot as plt
import math

'''++++++++++++++++++
+ CLASS DEFINITIONS +
++++++++++++++++++'''    
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



'''+++++++++++++++++++++++++++
+ GLOBAL CONSTANT PARAMETERS +
+++++++++++++++++++++++++++'''    
COMPLEX = None
GRAVITY = 1.985

''' Goal Func Param '''
GOAL_MULT   = 1.75  # g_mult parameter
GOAL_SIGMA  = -3.0  # g_sig parameter 
GOAL_GAMMA  = 25.0  # g_gam parameter
GOAL_LIMIT  = 30.0  # g_lim parameter
#######################################
#+###-------# NON COMPLEX #-------###+#
#######################################
GOAL_MULT2  = 6.0  # g_mult parameter
GOAL_LIMIT2 = 30.0  # g_lim parameter
GOAL_MAX    = 4.0   # g_max parameter 

''' Danger Func Param '''
DANGER_MULT     = 1.75  # d_mult parameter
DANGER_TO_CLOSE = 8.0   # d_tc parameter
DANGER_SIGMA    = 3.85  # d_sig parameter
DANGER_GAMMA    = 18.5  # d_gam parameter
DANGER_ALPHA    = 20.0  # d_alf parameter
DANGER_CUT_OFF  = 5.0   # d_co parameter
DANGER_LIMIT    = 40.0  # d_lim parameter
##########################################
#+###--------# NON  COMPLEX #--------###+#
##########################################
DANGER_MULT2    = 3.5   # d_mult parameter
DANGER_LIMIT2   = 40.0  # d_lim parameter
DANGER_CUT_OFF2 = 5.0   # d_co parameter
DANGER_MAX      = 4.0   # d_max parameter 

''' Drone Attraction Func Param '''
DRONE_ATT_MULT     = 4.0   # da_mult parameter
SWARM_DIST         = 12.0  # da_sd parameter
DRONE_ATT_CUT_OFF  = 18.9  # da_co parameter
DRONE_ATT_LIMIT    = 40.0  # da_lim parameter  
##############################################
#+###----------# NON  COMPLEX #----------###+#
##############################################
DRONE_ATT_MULT2    = 4.0   # da_mult parameter
DRONE_ATT_LIMIT2   = 40.0  # da_lim parameter
DRONE_ATT_CUT_OFF2 = 20.0  # da_co parameter  

''' Drone Repulsion Func Param '''
DRONE_REP_MULT   = 4.0   # dr_mult parameter
DRONE_TO_CLOSE   = 4.0   # dr_dtc parameter
DRONE_REP_GAMMA  = 1.5   # dr_gam parameter
DRONE_REP_ALPHA  = 0.1   # dr_alf parameter
DRONE_REP_SIGMA  = 10.0  # dr_sig parameter
DRONE_REP_LIMIT  = 30.0  # dr_lim parameter     
############################################
#+###----------# NON  COMPLEX #--------###+#
############################################
DRONE_REP_MULT2  = 4.0   # dr_mult parameter
DRONE_REP_LIMIT2 = 30.0  # dr_lim parameter
DRONE_REP_MAX    = 3.0   # dr_sig parameter
     


'''+++++++++++++++++++++
+ FUNCTION DEFINITIONS +
+++++++++++++++++++++''' 
#calculate repulsion forces to other drones
def linear_Repulsion(pos:Point, target:Point, limit:float, sigma:float=None, 
                     gamma:float=None, alpha:float=None, maxVal:float=None)->Point:
  force = target-pos
  magnitude = force.mag()
  d = min(magnitude, limit)

  strength = max((sigma/(d+alpha)-gamma),0)*(-1) if COMPLEX else (-1)*maxVal*GRAVITY/max(1,d)

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
def linear_Attraction(pos:Point, target:Point, limit:float, sigma:float=None,
                      gamma:float=None, maxVal:float=None)->Point:
  force = target - pos
  magnitude = force.mag()
  d = min(magnitude, limit)

  strength = (GRAVITY/sigma)*d+gamma if COMPLEX else (GRAVITY*maxVal)/max(d,1)
  
  force = force.scale_up(strength/max(magnitude,1))
  return force

# calculate repulsion force with DANGER_VECTORS
def limExp_Repulsion(pos:Point, target:Point, limit:float, cutOff:float, sigma:float=None, 
                     gamma:float=None, alpha:float=None, maxVal:float=None)-> Point:
  force = target-pos
  magnitude = force.mag()
  d = min(magnitude, limit)
  
  if COMPLEX:
    strength = GRAVITY*(d/2-limit)/sigma if(d<cutOff)else (-1)*math.exp((-1)*(GRAVITY*(alpha*math.log(d)-gamma)/sigma))
  else:
    strength = (-1)*(GRAVITY*maxVal)/max(d,1) if(d<cutOff)else (-1)*math.exp((-1)*GRAVITY*math.log(d))

  force = force.scale_up(strength/max(magnitude,1))
  return force



'''+++++++++++++
+ MAIN RUNTIME +
+++++++++++++''' 
test_min = -60
test_max = 60
pos = Point(0,0,0)
targets = [Point(it,0,0) for it in range(test_min,test_max)]
goal_vals,danger_vals,d_att_vals,d_rep_vals,dists = ([],[],[],[],[])
goal_vals_c,danger_vals_c,d_att_vals_c,d_rep_vals_c = ([],[],[],[])

# create simple set
COMPLEX = False
for target in targets:
  goal_vals.append(linear_Attraction(pos,target,GOAL_LIMIT2,maxVal=GOAL_MAX).mag()*GOAL_MULT2)
  d_att_vals.append(log_Attraction(pos,target,DRONE_ATT_LIMIT2,DRONE_ATT_CUT_OFF2).mag()*DRONE_ATT_MULT2)
  negative = False
  for coord in linear_Repulsion(pos,target,DRONE_REP_LIMIT2,maxVal=DRONE_REP_MAX): 
    if coord<0: 
      negative = True
      break
  d_rep_vals.append(linear_Repulsion(pos,target,DRONE_REP_LIMIT2,maxVal=DRONE_REP_MAX).mag()*DRONE_REP_MULT2*(-1 if negative else 1))
  negative = False
  for coord in limExp_Repulsion(pos,target,DANGER_LIMIT2,DANGER_CUT_OFF2,maxVal=DANGER_MAX): 
    if coord<0: 
      negative = True
      break
  danger_vals.append(limExp_Repulsion(pos,target,DANGER_LIMIT2,DANGER_CUT_OFF2,maxVal=DANGER_MAX).mag()*DANGER_MULT2*(-1 if negative else 1))

  negative = False
  for coord in (target-pos): 
    if coord<0: 
      negative = True
      break
  dists.append((target-pos).mag()*(-1 if negative else 1))


fig1 = plt.figure(1)
plt.plot(dists,goal_vals,color='green',label="Goal-Magnitude")
plt.plot(dists,danger_vals,color='red',label="Danger-Magnitude")
plt.plot(dists,d_att_vals,color='cyan',label="Drone_ATT-Magnitude")
plt.plot(dists,d_rep_vals,color='orange',label="Drone_REP-Magnitude")
plt.legend()
fig1.suptitle("Simple Scoring Function Results")

fig2,axes = plt.subplots(2,2)
axes[0,0].plot(dists[test_min:test_min+int(math.ceil(GOAL_LIMIT2))],goal_vals[test_min:test_min+int(math.ceil(GOAL_LIMIT2))],color='green',label="Goal-Magnitude")
axes[0,1].plot(dists[test_min:test_min+int(math.ceil(DANGER_LIMIT2))],danger_vals[test_min:test_min+int(math.ceil(DANGER_LIMIT2))],color='red',label="Danger-Magnitude")
axes[1,0].plot(dists[test_min:test_min+int(math.ceil(DRONE_ATT_LIMIT2))],d_att_vals[test_min:test_min+int(math.ceil(DRONE_ATT_LIMIT2))],color='cyan',label="Drone_ATT-Magnitude")
axes[1,1].plot(dists[test_min:test_min+int(math.ceil(DRONE_REP_LIMIT2))],d_rep_vals[test_min:test_min+int(math.ceil(DRONE_REP_LIMIT2))],color='orange',label="Drone_REP-Magnitude")
for arr in axes: 
  for ax in arr: 
    ax.legend()
fig2.suptitle("Simple Limited Scoring Function Results")




# create complex set 
COMPLEX = True
for target in targets:
  goal_vals_c.append(linear_Attraction(pos,target,GOAL_LIMIT,GOAL_SIGMA,GOAL_GAMMA).mag()*GOAL_MULT)
  d_att_vals_c.append(log_Attraction(pos,target,DRONE_ATT_LIMIT,DRONE_ATT_CUT_OFF).mag()*DRONE_ATT_MULT)
  negative = False
  for coord in linear_Repulsion(pos,target,DRONE_REP_LIMIT,DRONE_REP_SIGMA,DRONE_REP_GAMMA,DRONE_REP_ALPHA): 
    if coord<0: 
      negative = True
      break
  d_rep_vals_c.append(linear_Repulsion(pos,target,DRONE_REP_LIMIT,DRONE_REP_SIGMA,DRONE_REP_GAMMA,DRONE_REP_ALPHA).mag()*DRONE_REP_MULT*(-1 if negative else 1))
  negative = False
  for coord in limExp_Repulsion(pos,target,DANGER_LIMIT,DANGER_CUT_OFF,DANGER_SIGMA,DANGER_GAMMA,DANGER_ALPHA): 
    if coord<0: 
      negative = True
      break
  danger_vals_c.append(limExp_Repulsion(pos,target,DANGER_LIMIT,DANGER_CUT_OFF,DANGER_SIGMA,DANGER_GAMMA,DANGER_ALPHA).mag()*DANGER_MULT*(-1 if negative else 1))


fig3 = plt.figure(3)
plt.plot(dists,goal_vals_c,color='green',label="Goal-Magnitude")
plt.plot(dists,danger_vals_c,color='red',label="Danger-Magnitude")
plt.plot(dists,d_att_vals_c,color='cyan',label="Drone_ATT-Magnitude")
plt.plot(dists,d_rep_vals_c,color='orange',label="Drone_REP-Magnitude")
plt.legend()
fig3.suptitle("Complex Scoring Function Results")

fig4,axes = plt.subplots(2,2)
axes[0,0].plot(dists[test_min:test_min+int(math.ceil(GOAL_LIMIT))],goal_vals_c[test_min:test_min+int(math.ceil(GOAL_LIMIT))],color='green',label="Goal-Magnitude")
axes[0,1].plot(dists[test_min:test_min+int(math.ceil(DANGER_LIMIT))],danger_vals_c[test_min:test_min+int(math.ceil(DANGER_LIMIT))],color='red',label="Danger-Magnitude")
axes[1,0].plot(dists[test_min:test_min+int(math.ceil(DRONE_ATT_LIMIT))],d_att_vals_c[test_min:test_min+int(math.ceil(DRONE_ATT_LIMIT))],color='cyan',label="Drone_ATT-Magnitude")
axes[1,1].plot(dists[test_min:test_min+int(math.ceil(DRONE_REP_LIMIT))],d_rep_vals_c[test_min:test_min+int(math.ceil(DRONE_REP_LIMIT))],color='orange',label="Drone_REP-Magnitude")
for arr in axes: 
  for ax in arr: 
    ax.legend()
fig4.suptitle("Complex Limited Scoring Function Results")

plt.show()