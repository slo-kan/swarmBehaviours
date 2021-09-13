# Swarm Behaviours
This is a repository where we are going to share our swarm behaviour implementations.

In the file [helpful_links.txt](helpful_links.txt) we are going to post links to websites, videos or documents, that can help us figuring out how to implement the behaviours.
This behaviours will all be implemented assuming that we have a working interface and enough knowledge about the API to receive and send the needed data.
Most C and Python Code will be specific to the Software Project that we are working on.
This will help migrating the code later on to the actual Software Project.

In this Repository you will also find a simplified [2D Java Demo](Java-Demo) of our Swarm Behaviors.<br>
To view the demo coded in Java you need to download an IDE called [Processing](https://processing.org/download/) and load the project

The file [testParameters.py](testParameters.py) is a tool, that helps you determine theoretically good hyperparameters for the different swarm behaviors - of course you should test them and see how they work in the simulation.<br><br><br>


## Swarm Tasks/Arena Scenarios
### *-> First Scenario: [flight_plan.cvs](Project-Code/Scenarios/flight_plan.cvs)*
This file resembles a simple flight plan scenario in which the swarm has to fly to different locations one after another by reaching goal/attraction points and avoiding repell points. The attraction point Corrdinates can be found in the first line of the file and the repell point coordinates can be found in the second line of the file. Each line in the file is representing an array of points seperated by a semicolon(;) and each point consist of latitude and longditude coordinates seperated by a comma(,). The coordinates are given in a floating point format with degrees as unit.

### *-> Second Scenario: [flight_plan_circle.cvs](Project-Code/Scenarios/flight_plan_circle.cvs)*
Each line is representing an array of points separated by semicolan(;) and each point has a latitude and longitude separtaed by comma(,) in the same order. First line consists of a repel point. Second line consists of 10 attraction points in a circle of radius 50 meters with repel point as center. Third line consists of 10 points in a circle of radius 5 meters with repel point as center to spawn the aircrafts.
