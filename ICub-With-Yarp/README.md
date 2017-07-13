# Intelligent-Robotics
In this assignment you will be using the iCub Simulator with YARP and code a piece of C++ software that shall be able to:

1. Read-out the image from the iCub camera video stream into your program (see lab sheet 3)
2. The simulator needs to look at the screen while a video/camera feed will be shown to it (see lab sheet 2.1,2.2).
3. Apply linear filter on these images, using openCV (see lecture 2 page 12-13), e.g. threshold for colors/color map or the sobel-derivatives, edge detection, show the results in yarpview windows (see lab sheet 1, 3).
4. Apply face detection on these images using openCV.
5. Apply an object detection system, e.g. the circle detection (see lecture 2 page 12-13) or a marker detection (e.g. AR_marker,http://docs.opencv.org/3.1.0/d5/dae/tutorial_aruco_detection.html).
6. Control the iCub_sim head with your program to show the focus of attention of the system (focus of attention should be arbitrated by a finite-state machine (FSM) or finite-state automaton (FSA, plural: automata), finite automaton, or simply a state machine, e.g. turn towards the circle/marker when it can be seen on your camera/video input, using e.g. the test_grabber (see iKinGazeControler/xd:i port, lab sheet 2.2, 3)  or toward the face you can detect when there is one.
7. Control the iCub_sim arm to wave and/or to present a gesture when it is seeing a particular circle or marker or face.
