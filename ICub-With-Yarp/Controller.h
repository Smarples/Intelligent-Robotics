//
// Created by nicholas on 14/02/17.
//

#ifndef COURSEWORK1_CONTROLLER_H
#define COURSEWORK1_CONTROLLER_H

#include <stdio.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <chrono>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv/cv.h"
#include "opencv2/objdetect/objdetect.hpp"
#include <opencv/cvaux.h>

using namespace std::chrono;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace cv;

typedef std::chrono::high_resolution_clock CTime;
typedef std::chrono::duration<float> fsec;

class Controller {
private:
    String face_cascade_name = "haarcascade_frontalface_alt.xml";
    String eyes_cascade_name = "haarcascade_eye_tree_eyeglasses.xml";
    CascadeClassifier face_cascade;
    CascadeClassifier eyes_cascade;
    //string window_name = "Image";
    //RNG rng(12345);
public:

    Network _yarp; // set up yarp
    BufferedPort<ImageOf<PixelRgb> > imagePort, imageOut, facePort;  // make a port for reading images

    yarp::sig::Vector setpointsHead;
    yarp::sig::Vector setpointsRARM;
    yarp::sig::Vector setpointsLARM;

    IPositionControl *posHead;
    IVelocityControl *velHead;
    IEncoders *encHead;
    IPositionControl *posRARM;
    IVelocityControl *velRARM;
    IEncoders *encRARM;
    IPositionControl *posLARM;
    IVelocityControl *velLARM;
    IEncoders *encLARM;

    Property optionsHead;
    Property optionsRARM;
    Property optionsLARM;

    int jntsHead;
    int jntsRARM;
    int jntsLARM;

    //Functions
    void waveCutRight(CTime::time_point t0);
    void waveCutLeft(CTime::time_point t0);
    void rightHandCountingToFive(int numberCircles, CTime::time_point t0);
    void leftHandCountingToFive(int numberCircles, CTime::time_point t0);
    void lookAround(CTime::time_point t0);
    void resetLARM();
    void resetRARM();
    void resetHEAD();
    void resetPos();
    Point checkScreenFace();
    int checkScreenLinear();
	int checkCircles();
	Mat getImage();
    void linear_filter();
    int init();
};


#endif //COURSEWORK1_CONTROLLER_H
