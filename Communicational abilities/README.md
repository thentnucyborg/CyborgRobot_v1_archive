The software in this folder performs speech recognition and generates answers to questions asked. 
In order to include the ability for the robot to speak the answer back(text to speech), the software
for the trollface written by Håvard Svoen has to be included. That is, the software controlling the trollface. 

To do this, include the ROS package from the work done by Svoen within the same directory as the "speech_processing" 
package, and compile by using "catkin_workspace" within the directory given the name "CyborgWorkspace". 

In order for the software to work, audio hardware is required. That is, a microphone. 

All modules are located in the directory "CyborgWorkspace/src/speech_processing/scripts". 
The reason for the directory setup is that catkin has been used to build the project. 



Two wiki pages have been written to explain the software. These are:

- https://www.ntnu.no/wiki/display/cyborg/j.Giving+NTNU+Cyborg+Communicational+Abilities
- https://www.ntnu.no/wiki/display/cyborg/k.Cyborg\%27s+Speech+Recognition+Solution

The libraries and functionality are included in the wiki, including a demonstration video showing an extract
from the results obtained. 

An additional library called "Miscellaneous Software" contains software used for plotting response time
of the several modules implemented, in addition to software used for comparing the response time of the 
speech recognition APIs. 


Contact information in case of questions:

Steinar Kraugerud
steinar.kraugerud@gmail.com
+47 97 12 37 36

If there are any issues, do not hesitate to contact me:)

