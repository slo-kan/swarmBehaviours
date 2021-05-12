// assume we have a position array of 50 positions (of other aircrafts) and the 
// position of our own aircraft how do we implement the attraction repulsion 
// swarm behaviour for a copter

// given this:

 typedef struct {
     double x;
     double y;
     double z;
 } Pos;

 int droneCount = 50
 Pos otherPos[droneCount];
 Pos ownPos;
 Pos perlimiter;
 
// implement the swarm behaviour

Pos attract()
{
    Pos targetPos = {0,0,0};
    for(int idx=0; idx<droneCount; ++idx)
    {
      targetPos.x += otherPos[idx].x-ownPos.x;
      targetPos.y += otherPos[idx].y-ownPos.y;
      targetPos.z += otherPos[idx].z-ownPos.z;
    }
    return targetPos;
}

Pos repulse()
{
    Pos targetPos = {0,0,0};
    Pos offsetPos = {0,0,0};
    for(int idx=0; idx<droneCount; ++idx)
    {
      offsetPos.x = otherPos[idx].x-ownPos.x;
      if(offsetPos.x<perlimiter.x)
        targetPos.x -= offsetPos.x-perlimiter.x;
      offsetPos.y = otherPos[idx].y-ownPos.y;
      if(offsetPos.y<perlimiter.y)
        targetPos.y -= offsetPos.y-perlimiter.y;
      offsetPos.y = otherPos[idx].y-ownPos.y;
      if(offsetPos.z<perlimiter.z)
        targetPos.z -= offsetPos.z-perlimiter.z;
    }
    return targetPos;
}


// this is most likely far from correct please feel free to fix or override this - this is just to give you an idea of how this code could look like
