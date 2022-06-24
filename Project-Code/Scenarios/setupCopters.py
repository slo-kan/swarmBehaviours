#!/usr/bin/python

#module imports
import sys,copy
import xml.etree.ElementTree as Parser
from os import path

SCENARIO_DIR = path.dirname(path.abspath(__file__))
CONF_DIR = path.normpath(path.join(SCENARIO_DIR, '../../../../conf'))
COPTER_DIR = path.normpath(path.join(CONF_DIR, 'airframes/ovgu'))
FLIGHT_PLAN_DIR = path.normpath(path.join(CONF_DIR, 'flight_plans/ovgu_swarm'))


def setup_attrep_copter_conf(amounts):
  FIRST_ATT_ID:int = 2 #this value influences behaviour across multiple files please be sure to change it everywhere if you change it here
  FIRST_SWARM_ID:int = 30 #this value influences behaviour across multiple files please be sure to change it everywhere if you change it here
  tree = Parser.parse((COPTER_DIR+"/npsAttRepCopter.xml"))
  root = tree.getroot()
  others = []
  for module in root.iter("module"):
    if module.get("name")=="attRep":
      for subelement in module.iter():
        if not(subelement.tag == "module" or \
          (subelement.tag == "define" and ("ATTRACTION" in subelement.get("name") or \
          "REPELL" in subelement.get("name") or "SWARM_MEMBER" in subelement.get("name")))):
          others.append(copy.deepcopy(subelement))
      module.clear()
      module.set("name","attRep")
      Parser.SubElement(module,"define",{"name":"FIRST_ATTRACTION_POINT_ID","value":str(FIRST_ATT_ID)})
      Parser.SubElement(module,"define",{"name":"LAST_ATTRACTION_POINT_ID","value":str(FIRST_ATT_ID+amounts["att"])})
      Parser.SubElement(module,"define",{"name":"FIRST_REPELL_POINT_ID","value":str(FIRST_ATT_ID+amounts["att"])})
      Parser.SubElement(module,"define",{"name":"LAST_REPELL_POINT_ID","value":str(FIRST_ATT_ID+amounts["att"]+amounts["rep"])})
      Parser.SubElement(module,"define",{"name":"FIRST_SWARM_MEMBER_ID","value":str(FIRST_SWARM_ID)})
      Parser.SubElement(module,"define",{"name":"LAST_SWARM_MEMBER_ID","value":str(FIRST_SWARM_ID+amounts["spawn"]-1)})
      for subelement in others: module.append(subelement)
      break
  tree.write((COPTER_DIR+"/npsAttRepCopter.xml"))

def setup_consteer_copter_conf(amounts):
  FIRST_GOAL_ID:int = 2 #this value influences behaviour across multiple files please be sure to change it everywhere if you change it here
  FIRST_SWARM_ID:int = 50 #this value influences behaviour across multiple files please be sure to change it everywhere if you change it here
  tree = Parser.parse((COPTER_DIR+"/npsConSteerCopter.xml"))
  root = tree.getroot()
  others = []
  for module in root.iter("module"):
    if module.get("name")=="conSteer":
      for subelement in module.iter():
        if not(subelement.tag == "module" or \
          (subelement.tag == "define" and ("GOAL" in subelement.get("name") or \
          "DANGER" in subelement.get("name") or "SWARM_MEMBER" in subelement.get("name")))):
          others.append(copy.deepcopy(subelement))
      module.clear()
      module.set("name","conSteer")
      Parser.SubElement(module,"define",{"name":"FIRST_GOAL_POINT_ID","value":str(FIRST_GOAL_ID)})
      Parser.SubElement(module,"define",{"name":"LAST_GOAL_POINT_ID","value":str(FIRST_GOAL_ID+amounts["att"])})
      Parser.SubElement(module,"define",{"name":"FIRST_DANGER_POINT_ID","value":str(FIRST_GOAL_ID+amounts["att"])})
      Parser.SubElement(module,"define",{"name":"LAST_DANGER_POINT_ID","value":str(FIRST_GOAL_ID+amounts["att"]+amounts["rep"])})
      Parser.SubElement(module,"define",{"name":"FIRST_SWARM_MEMBER_ID","value":str(FIRST_SWARM_ID)})
      Parser.SubElement(module,"define",{"name":"LAST_SWARM_MEMBER_ID","value":str(FIRST_SWARM_ID+amounts["spawn"]-1)})
      for subelement in others: module.append(subelement)
      break
  tree.write((COPTER_DIR+"/npsConSteerCopter.xml"))


def setup_control_panel(file_name:str):
  tree = Parser.parse((CONF_DIR+"/control_panel.xml"))
  root = tree.getroot()
  file_path = "Scenarios/"+file_name.split("_swarm_positions.csv")[0]+".csv"
  for session in root.iter("session"):
    if "Swarm_" in session.get("name"):
      for program in session.iter("program"):
        if "WP_Move"==program.get("name"):
          for arg in program.iter("arg"):
            if "Scenarios" in arg.get("constant"):
              arg.set("constant",file_path)
    elif "Initialize" == session.get("name"):
      arg = session.find("program").find("arg")
      arg.set("constant",file_name)
  tree.write((CONF_DIR+"/control_panel.xml"))


def setup_flight_plans(file_name:str):
  amounts = {}
  with open((SCENARIO_DIR+'/'+file_name),"r") as inputFile:
    lines = inputFile.readlines()
    values = lines[0].strip().split(",")
    for val in values: amounts[val.strip().split(":")[0]] = int(val.strip().split(":")[1])
    print(amounts)
    coordinate_values = lines[2:]
    print(coordinate_values)
    for ac_id,coords in enumerate(coordinate_values):
      coords = coords.strip()
      if not coords: continue
      tree = Parser.parse((FLIGHT_PLAN_DIR+("/member%d.xml"%ac_id)))
      root = tree.getroot()
      coords = coords.split(",")
      root.set("lat0", coords[0].strip())
      root.set("lon0", coords[1].strip())
      waypoints = root.find("waypoints")
      others = []
      for waypoint in waypoints.iter("waypoint"): 
        if not(waypoint.get("name")=="FOLLOW" or \
            ("ATTRACTION" in waypoint.get("name")) or \
            ("REPELL" in waypoint.get("name"))):
          others.append(copy.deepcopy(waypoint))
      print(others)
      waypoints.clear()
      Parser.SubElement(waypoints,"waypoint",{"name":"FOLLOW","x":"0.0","y":"0.0"})
      for att_id in range(amounts["att"]): 
        name:str = "ATTRACTION_POINT_%d"%att_id
        Parser.SubElement(waypoints,"waypoint",{"name":name,"x":"0.0","y":"0.0"})
      for rep_id in range(amounts["rep"]): 
        name:str = "REPELL_POINT_%d"%rep_id
        Parser.SubElement(waypoints,"waypoint",{"name":name,"x":"0.0","y":"0.0"})
      for waypoint in others: waypoints.append(waypoint)
      tree.write((FLIGHT_PLAN_DIR+("/member%d.xml"%ac_id)))
      content = ""
      with open((FLIGHT_PLAN_DIR+("/member%d.xml"%ac_id)),"r") as file: content = file.read()
      content = content.replace("&gt;",">")
      content = content.replace("&lt;","<")
      with open((FLIGHT_PLAN_DIR+("/member%d.xml"%ac_id)),"w") as file: file.write(content)
  return amounts
        

#main program
if __name__=='__main__' and (len(sys.argv)>1):
  amounts = setup_flight_plans(sys.argv[1])
  setup_attrep_copter_conf(amounts)
  setup_consteer_copter_conf(amounts)
  setup_control_panel(sys.argv[1])