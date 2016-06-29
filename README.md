DVS128-Robot-Control
====================
My masters project at Edinburgh University involves a custom-written robot navigation algorithm for a BioLoid Premium robot using the DVS128 asynvchronous, neuromorphic vision sensor (details here: siliconretina.ini.uzh.ch/ ).

This software requires the jAER framework, which can be found here: http://sourceforge.net/apps/trac/jaer/wiki. 

This is heavilly customized code for a specific scenario, based on experiments by Srinivasan on honeybees (see http://jeb.biologists.org/content/199/1/237)

The control algorithm was based on and adapted from this paper: http://cognet.mit.edu/journal/10.1162/089976699300016700 , though my algorithm is unique in it using an asynchronous, non-frame-based camera, the DVS128 (so, further modifications were done)

The following directory structure is included:
1. divers/ - The driver for the DVS128 retina, located in drivers/ - I've included both the Windows and Linux drivers, and a readme, as downloaded from the jaer website (abovementioned);
2. lib/ - All libraries, used by Eclipse to start the jaer software. 
3. robotcontrol/ - Contains the filter application that runs on top of JAer, along with custom logic for creating and updating the image data, and the actual controling algorithm which uses the visual data to navigate the robot;
4. neuron/ - The Java-implementation of a neuron network, used for depth extraction and wall detection;
5. RobotControl_v_1_0/ - This is the code that runs on the robot's controller, written in C;
6. relay/ - This code, written in C, performs two-way communication between the computer, on which the JAer filter and custom controller are working, and the Robot. Communication is done via ZigBee Wireless;
7. masters/ - Contains my Master's Thesis;


