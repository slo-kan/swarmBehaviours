import matplotlib.pyplot as plt
import sys, getopt, math

# standard constant values
GRAVITY = 1.985              
#LINEAR_MULTIPLIER = 0.8      
#COMFY_MULTIPLIER = 1.25      
#SIMP_EXP_MULTIPLIER = 3.0    
#COMP_EXP_MULTIPLIER = 1.5    
#COMFY_DISTANCE = 17         
#PERLIMITER = 6.425           

# Att_Rep scaled bei 1/10
LINEAR_MULTIPLIER = 4.4      
COMFY_MULTIPLIER = 0.3
SIMP_EXP_MULTIPLIER = 4.4
COMP_EXP_MULTIPLIER = 1.0
COMFY_DISTANCE = 4.8
PERLIMITER = 2.4


def linAtt_compExpRep(ka,kb,r,x):
    c = (r*r)/(math.log(10*kb/ka))
    return (-x)*((-ka/10)-(kb*math.exp(-(x*x)/c)))/2

def linAtt_simExpRep(ka,kb,r,x):
    return(-x)*((-ka)*(kb*math.exp(-(x*x)/(2*r*r))))/10

def comfyAtt_compExpRep(ka,kb,r,d,x):
    c = (r*r)/(math.log(kb/ka))
    return (-x)*((-ka*(abs(x)-d)/max(abs(x),0.01))-(kb*math.exp(-(x*x)/c)))

def comfyAtt_simExpRep(ka,kb,r,d,x):
    return (-x)*((-ka*(abs(x)-d)/max(abs(x),0.01))-(kb*math.exp(-(x*x)/(2*r*r))))

def linAtt(ka,x):
    x = max(1,abs(x))
    return (-ka)/(-x)

def simpExpRep(ka,r,x):
    return (-x)*(-ka)*math.exp(-(x*x)/(2*r*r))            
                    

# command line argument  
def main(argv):
   (g,lm,cm,em,eb,d,r) = (None,None,None,None,None,None,None)
   try:
      opts, _ = getopt.getopt(argv,"hg:l:c:e:b:d:r:",["help","gravity=","linear_mult=","comfy_mult=","simp_exp_mult=","comp_exp_mult=","comfy_dist=","perlimiter"])
      for opt, arg in opts:
          if opt in ("-h", "--help"):
             print('testParameters.py -g <GRAVITY> -l <MULTIPLIER> -c <MULTIPLIER> -e <MULTIPLIER> -b <MULTIPLIER> -d <COMFY_DISTANCE> -r <PERLIMITER>')
             sys.exit()
          elif opt in ("-g", "--gravity"):
             g = float(arg)
          elif opt in ("-l", "--linear_mult"):
             lm = float(arg)
          elif opt in ("-c", "--comfy_mult"):
             cm = float(arg)
          elif opt in ("-e", "--simp_exp_mult"):
             em = float(arg)
          elif opt in ("-b", "--comp_exp_mult"):
             eb = float(arg)
          elif opt in ("-d", "--comfy_dist"):
             d = float(arg)
          elif opt in ("-r", "--perlimiter"):
             r = float(arg)
   except getopt.GetoptError:
      print('testParameters.py -g <GRAVITY> -l <MULTIPLIER> -c <MULTIPLIER> -e <MULTIPLIER> -b <MULTIPLIER> -d <COMFY_DISTANCE> -r <PERLIMITER>')
      sys.exit(2)

   # user input if no argument line arguments
   if(not(g and lm and cm and em and eb and d and r)): print('Manual value assignment skip through with enter key for standard values!')
   if(not(g)):
       try:
           num = float(input("Enter value for GRAVITY: "))
           g = num
       except:
           g = GRAVITY
   if(not(lm)):
       try:
           num = float(input("Enter value for LINEAR_MULTIPLIER: "))
           lm = num
       except:
           lm = LINEAR_MULTIPLIER
   if(not(cm)):
       try:
           num = float(input("Enter value for COMFORTABLE_DISTANCE_MULTIPLIER: "))
           cm = num
       except:
           cm = COMFY_MULTIPLIER
   if(not(em)):
       try:
           num = float(input("Enter value for SIMPLE_EXPONANTIAL_MULTIPLIER: "))
           em = num
       except:
           em = SIMP_EXP_MULTIPLIER
   if(not(eb)):
       try:
           num = float(input("Enter value for COMPLEX_EXPONANTIAL_MULTIPLIER: "))
           eb = num
       except:
           eb = COMP_EXP_MULTIPLIER
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
   linMult = g*lm
   comfyMult = g*cm
   simExpMult = g*em
   comExpMult = g*eb

   fives = []
   for num in range(-10,10): fives.append(num/2)
   distances = [*range(-30,-10,2),*range(-10,-5,1),*fives,*range(5,10,1),*range(10,30,2)]
   (y1,y2,y3,y4,y5,y6) = ([],[],[],[],[],[])
   for x in distances:
       y1.append(linAtt_compExpRep(linMult,comExpMult,r,x))
       y2.append(linAtt_simExpRep(linMult,simExpMult,r,x))
       y3.append(comfyAtt_compExpRep(comfyMult,comExpMult,r,d,x))
       y4.append(comfyAtt_simExpRep(comfyMult,simExpMult,r,d,x))
       y5.append(linAtt(linMult,x))
       y6.append(simpExpRep(simExpMult,r,x))

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

   fig = plt.figure(3)
   plt.xlabel('distance')
   plt.ylabel('force')
   plt.plot(distances, y5, 'tab:green')
   plt.plot(distances, y6, 'tab:red')
   plt.plot(distances, y3, 'tab:blue')
   plt.legend(["linAtt","simExpRep","comfyAtt_compExpRep"])
   plt.show()
    

if __name__ == "__main__":
   main(sys.argv[1:])
