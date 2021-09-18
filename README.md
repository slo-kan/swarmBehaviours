# Swarm Behaviours
This is a repository where we are going to share our swarm behaviour implementations.

In the file [helpful_links.txt](helpful_links.md) we are going to post links to websites, videos or documents, that can help us figuring out how to implement the behaviours.
This behaviours will all be implemented assuming that we have a working interface and enough knowledge about the API to receive and send the needed data.
Most C and Python Code will be specific to the Software Project that we are working on.
This will help migrating the code later on to the actual Software Project.

In this Repository you will also find a simplified [2D Java Demo](Java-Demo) of our Swarm Behaviors.<br>
To view the demo coded in Java you need to download an IDE called [Processing](https://processing.org/download/) and load the project

The file [testParameters.py](testParameters.py) is a tool, that helps you determine theoretically good hyperparameters for the different swarm behaviors - of course you should test them and see how they work in the simulation.<br><br><br>


## Swarm Tasks/Arena Scenarios
### *-> First Scenario: [flight_plan.csv](Project-Code/Scenarios/flight_plan.csv)*
This file resembles a simple flight plan scenario in which the swarm has to fly to different locations one after another by reaching goal/attraction points and avoiding repell points. The repell point corrdinates, attraction point coordinates and spawn point coordinates can be found each in one line. The first row gives each collumn a name. Coordinates can also be found [here](https://www.google.com/maps/d/u/0/edit?mid=1aDNCp8DNH7VNPh4EjR8B5AbEjCzVExEa&usp=sharing).

### *-> Second Scenario: [flight_plan_circle.csv](Project-Code/Scenarios/flight_plan_circle.csv)*
This file resembles a simple flight plan scenario in which the swarm is placed in a cirlce 5 meters away from a repell point, which is in the center of the scene. Also 10 attraction points set in a circle with a radius of 50 meters around the repell point. The first row gives each collumn a name. Coordinates can also be found [here](https://www.google.com/maps/d/u/0/edit?mid=1aDNCp8DNH7VNPh4EjR8B5AbEjCzVExEa&usp=sharing).

### *-> Third Scenario: [flight_plan_tube.csv](Project-Code/Scenarios/flight_plan_tube.csv)*
This file resembles a simple flight plan scenario in which the swarm has to fly through a tube of repell points to a horde of attraction points at the other side of the tube. The  10 repell point corrdinates, attraction point coordinates and spawn point coordinates can be found each in one line. The first row gives each collumn a name. Coordinates can also be found [here](https://www.google.com/maps/d/u/0/edit?mid=1aDNCp8DNH7VNPh4EjR8B5AbEjCzVExEa&usp=sharing).
