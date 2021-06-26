import matplotlib.pyplot as plt
import sys, getopt, math

# standard constant values
GRAVITY = 1.985
MULTIPLIER = 2.5
COMFY_DISTANCE = 5.25
PERLIMITER = 2.5


def linAtt_compExpRep(ka,kb,r,x):
    c = (r*r)/(math.log(kb/ka))
    return (-x)*((-ka)-(kb*math.exp(-(x*x)/c)))

def linAtt_simExpRep(ka,kb,r,x):
    return (-x)*((-ka)-(kb*math.exp(-(x*x)/(2*r*r))))

def comfyAtt_compExpRep(ka,kb,r,d,x):
    c = (r*r)/(math.log(kb/ka))
    return (-x)*((-ka*(abs(x)-d)/max(abs(x),0.01))-(kb*math.exp(-(x*x)/c)))

def comfyAtt_simExpRep(ka,kb,r,d,x):
    return (-x)*((-ka*(abs(x)-d)/max(abs(x),0.01))-(kb*math.exp(-(x*x)/(2*r*r))))                    
                    

# command line argument  
def main(argv):
   (g,m,d,r) = (None,None,None,None)
   try:
      opts, args = getopt.getopt(argv,"hg:m:d:r:",["help","gravity=","multiplier=","comfy_dist=","perlimiter"])
      for opt, arg in opts:
          if opt in ("-h", "--help"):
             print('testParameters.py -g <GRAVITY> -m <MULTIPLIER> -d <COMFY_DISTANCE> -r <PERLIMITER>')
             sys.exit()
          elif opt in ("-g", "--gravity"):
             g = float(arg)
          elif opt in ("-m", "--multiplier"):
             m = float(arg)
          elif opt in ("-d", "--comfy_dist"):
             d = float(arg)
          elif opt in ("-r", "--perlimiter"):
             r = float(arg)
   except getopt.GetoptError:
      print('testParameters.py -g <GRAVITY> -m <MULTIPLIER> -d <COMFY_DISTANCE> -r <PERLIMITER>')
      sys.exit(2)

   # user input if no argument line arguments
   if(not(g and m and d and r)): print('Manual value assignment skip through with enter key for standard values!')
   if(not(g)):
       try:
           num = float(input("Enter value for GRAVITY: "))
           g = num
       except:
           g = GRAVITY
   if(not(m)):
       try:
           num = float(input("Enter value for MULTIPLIER: "))
           m = num
       except:
           m = MULTIPLIER
   if(not(d)):
       try:
           num = float(input("Enter value for COMFY_DISTANCE: "))
           d = num
       except:
           d = COMFY_DISTANCE
   if(not(r)):
       try:
           num = float(input("Enter value for PERLIMITER: "))
           r = num
       except:
           r = PERLIMITER
    

   # calculate with formulars and some nicely chosen distance values
   ka = g
   kb = g*m

   fives = []
   for num in range(-10,10): fives.append(num/2)
   distances = [*range(-30,-10,2),*range(-10,-5,1),*fives,*range(5,10,1),*range(10,30,2)]
   (y1,y2,y3,y4) = ([],[],[],[])
   for x in distances:
       y1.append(linAtt_compExpRep(ka,kb,r,x))
       y2.append(linAtt_simExpRep(ka,kb,r,x))
       y3.append(comfyAtt_compExpRep(ka,kb,r,d,x))
       y4.append(comfyAtt_simExpRep(ka,kb,r,d,x))

   # plot force distance graph
   fig = plt.figure(1)
   gs = fig.add_gridspec(2, 2, hspace=0.4, wspace=0.2)
   axs = gs.subplots()
   axs[0][0].set_title("linAtt_compExpRep")
   axs[0][0].set(ylabel='force')
   axs[0][0].plot(distances, y1, 'tab:blue')
   axs[0][1].set_title("linAtt_simExpRep")
   axs[0][1].plot(distances, y2, 'tab:cyan')
   axs[1][0].set_title("comfyAtt_compExpRep")
   axs[1][0].set(xlabel='distance', ylabel='force')
   axs[1][0].plot(distances, y3, 'tab:orange')
   axs[1][1].set_title("comfyAtt_simExpRep")
   axs[1][1].set(xlabel='distance')
   axs[1][1].plot(distances, y4, 'tab:red')
   
   fig = plt.figure(2)
   plt.xlabel('distance')
   plt.ylabel('force')
   plt.plot(distances, y1, 'tab:blue')
   plt.plot(distances, y2, 'tab:cyan')
   plt.plot(distances, y3, 'tab:orange')
   plt.plot(distances, y4, 'tab:red')
   plt.legend(["linAtt_compExpRep","linAtt_simExpRep","comfyAtt_compExpRep","comfyAtt_simExpRep"])
   plt.show()
    

if __name__ == "__main__":
   main(sys.argv[1:])
