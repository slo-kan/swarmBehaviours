// assume we have a position array of 50 positions (of other aircrafts) and the 
// position of our own aircraft how do we implement the attraction repulsion 
// swarm behaviour for a copter

// given this:

 typedef struct {
     double x;
     double y;
     double z;
 } Pos;

 Pos otherPos[50];
 Pos ownPos;
 Pos perlimiter;
 
// implement the swarm behaviour

Pos attract()
{
    Pos targetPos = {0,0,0};
    for(int idx=0; idx<50; ++idx)
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
    for(int idx=0; idx<50; ++idx)
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