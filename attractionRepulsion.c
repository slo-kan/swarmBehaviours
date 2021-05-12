// assume we have a position array of 50 positions (of other aircrafts) and the 
// position of our own aircraft how do we implement the attraction repulsion 
// swarm behaviour for a copter

// given this:
 
 // structure for potentionally cleaner code
 typedef struct {
     double x;
     double y;
     double z;
 } Pos;

 // probably defined inline in the .h file as part of the api
 void setWayPoint(Pos position){}

 // provided through paparazzi interface
 #define droneCount 50
 Pos otherPos[droneCount];
 Pos ownPos;

 // tunable global parameters
 Pos perlimiter = {3,3,3};  //radius around drone where repulsion kicks in
 int gravity = 40;          //balance of 40% attractive gravity 
                            //and 60% repulsive gravity
 
// Implementation the swarm behaviour 
// ----------------------------------

// returns weighted attraction force vector
Pos attract()
{
    Pos targetPos = {0,0,0};
    for(int idx=0; idx<droneCount; ++idx)
    {
      targetPos.x += (gravity/((otherPos[idx].x-ownPos.x)*
                               (otherPos[idx].x-ownPos.x)))*
                      otherPos[idx].x;
      targetPos.y += (gravity/((otherPos[idx].y-ownPos.y)*
                              (otherPos[idx].y-ownPos.y)))*
                      otherPos[idx].y;
      targetPos.z += (gravity/((otherPos[idx].z-ownPos.z)*
                              (otherPos[idx].z-ownPos.z)))*
                      otherPos[idx].z;
    }
    return targetPos;
}

// returns weighted repulsive force vector
// repulsion is only active if perlimiter is hurt
Pos repulse()
{
    Pos targetPos = {0,0,0};
    Pos offsetPos = {0,0,0};
    for(int idx=0; idx<droneCount; ++idx)
    {
      offsetPos.x = otherPos[idx].x-ownPos.x;
      if(offsetPos.x<perlimiter.x)
        targetPos.x -= ((100-gravity)/(offsetPos.x*offsetPos.x))*
                        otherPos[idx].x;
      offsetPos.y = otherPos[idx].y-ownPos.y;
      if(offsetPos.y<perlimiter.y)
        targetPos.y -= ((100-gravity)/(offsetPos.y*offsetPos.y))*
                        otherPos[idx].y;
      offsetPos.z = otherPos[idx].z-ownPos.z;
      if(offsetPos.z<perlimiter.z)
        targetPos.z -= ((100-gravity)/(offsetPos.z*offsetPos.z))*
                        otherPos[idx].z;
    }
    return targetPos;
}

void updatePos()
{
  Pos attraction = attract();
  Pos repulsion = repulse();
  ownPos.x += attraction.x + repulsion.x;
  ownPos.y += attraction.y + repulsion.y;
  ownPos.z += attraction.z + repulsion.z; 
  setWayPoint(ownPos);
}

// this is most likely far from correct please feel free to fix or override 
// this code - this is just a quickly coded template to give you an idea of
// how this code could look like - please also feel free to make better use
// of the defined structure #OperatorOverloading
