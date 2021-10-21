# ECE_Map_SeniorDesign
Robotic Delivery System for UNCC Campus: Map generation and Path Planning (UNCC_ECE_MAP) 

## Description 
[Link](https://isl.charlotte.edu/sites/isl.charlotte.edu/files/media/UNCC%20ECE%20-%20UNCC_ECE_MAP%20-%20Fall%202021.pdf) to the detail documentation

This project is a property of *Unversity of North Carolina at Charlotte Electrical and Computer Engineering Deparment*
#### Company and Project Overview: 
> This project will be developed in the Control Systems and Autonomous Robots (CSAR) lab in the Electrical
and Computer Engineering Department at the University of North Carolina at Charlotte under the supervision
of the Principal Investigator (PI) Dipankar Maity. In this project, the students are required to develop a
robotic autonomous delivery system for UNCC campus where robots will be autonomously dispatched
from one location of the campus to go to another location while carrying light-weight materials such as books,
mails, small packages etc. This project will be done in two phases. Phase-1 with a senior design team will
consist of designing a (GPS-enabled) campus navigation system which will produce a planned path for the
robots to go from any campus location to any other location. The second phase, which will be build upon the
development of this phase, will be on the implementation of the algorithms into real robots.


## Table of content
#### Python files
- Dijkstras.py
- UNCEngineerMap.py
- get_data.py
- plot.py

#### Data
- [test code](https://github.com/leeinfy/ECE_Map_SeniorDesign/tree/main/test_code)
- [gpx](https://github.com/leeinfy/ECE_Map_SeniorDesign/tree/main/gpx)
- [csv](https://github.com/leeinfy/ECE_Map_SeniorDesign/tree/main/csv)

## Dijkstras.py
Dijkstras.py is the main file that will be used for the robot implementation. The Dijkstras file is currently responsible for setting up the nodes and their connections, running the dijkstra's algorithm, and displaying a graph of the path taken. These functions may be broken down into multiple files if that would allow for easier / more simple implementations when using the Turtlebots, but for now have been consolidated into a single file.

#### TODO: 
- Continue to expand the node network to include all of EPIC side of campus
- Implement paths to buildings that utilize the ramps


## GPX Data
The data files obtained from OSM that contain the coordinates along various road segments, these are loaded and interpreted and used to create the abstracted nodes in the node network.

Credit to [OSM](https://www.openstreetmap.org/copyright)
© OpenStreetMap contributors

## test_code
This folder includes all old code that was either used for testing purposes or was later used in the final implementation. Mostly just kept for reference or for smaller scale examples of the code. Also serves as a good folder to place code that uses alternative methods, or code that is not currently being used, but may be helpful in the future.
