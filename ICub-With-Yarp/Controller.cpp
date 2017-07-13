//
// Created by nicholas on 14/02/17.
//

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

#include "State.h"
#include "StateMachine.h"
#include "Controller.h"

typedef std::chrono::high_resolution_clock CTime;
typedef std::chrono::duration<float> fsec;

void Controller::waveCutRight(CTime::time_point t0) {
    auto t1 = CTime::now();
    fsec fs = t1 - t0;
    float elbowangle = 75 + 30 * sin(fs.count() * 2);

    setpointsRARM[0] = -60  + 4 * sin(fs.count()/3);
    setpointsRARM[1] = 75 + 4 * cos(fs.count()/2);
    setpointsRARM[2] = 0;
    setpointsRARM[3] = elbowangle;
    setpointsRARM[4] = 0;
    setpointsRARM[5] = 0;
    setpointsRARM[6] = 0;
    setpointsRARM[7] = 50;
    setpointsRARM[8] = 20;
    setpointsRARM[9] = 70;
    setpointsRARM[10] = 20;
    setpointsRARM[11] = 10;
    setpointsRARM[12] = 10;
    setpointsRARM[13] = 10;
    setpointsRARM[14] = 10;
    setpointsRARM[15] = 10;

    posRARM->setPositionMode();
    posRARM->positionMove(setpointsRARM.data());
}

void Controller::waveCutLeft(CTime::time_point t0) {
    auto t1 = CTime::now();
    fsec fs = t1 - t0;

    float elbowangle = 75 + 30 * sin(fs.count() * 2);

    setpointsLARM[0] = -60  + 4 * sin(fs.count()/3);
    setpointsLARM[1] = 75 + 4 * cos(fs.count()/2);
    setpointsLARM[2] = 0;
    setpointsLARM[3] = elbowangle;
    setpointsLARM[4] = 0;
    setpointsLARM[5] = 0;
    setpointsLARM[6] = 0;
    setpointsLARM[7] = 50;
    setpointsLARM[8] = 20;
    setpointsLARM[9] = 70;
    setpointsLARM[10] = 20;
    setpointsLARM[11] = 10;
    setpointsLARM[12] = 10;
    setpointsLARM[13] = 10;
    setpointsLARM[14] = 10;
    setpointsLARM[15] = 10;

    posLARM->setPositionMode();
    posLARM->positionMove(setpointsLARM.data());
}

void Controller::rightHandCountingToFive(int numberCircles, CTime::time_point t0) {
	auto t1 = CTime::now();
	fsec fs = t1 - t0;

	setpointsRARM[0] = -60 + 4 * sin(fs.count()/3);
	setpointsRARM[1] = 75 + 4 * cos(fs.count()/2);
	setpointsRARM[2] = 0;
	setpointsRARM[3] = 90;
	setpointsRARM[4] = 0;
	setpointsRARM[5] = 0;
	setpointsRARM[6] = 0;
	setpointsRARM[7] = 50;
	setpointsRARM[8] = 20;
	setpointsRARM[9] = 70;

	if (numberCircles ==5 )
		setpointsRARM[10] = 10; // THUMB 1-4:180, 5:10
	else
		setpointsRARM[10] = 180;

	setpointsRARM[11] = 10;

	if (numberCircles == 3 )
		setpointsRARM[12] = 180; // INDEX 1,2,4,5:10;  3: 180;
	else
		setpointsRARM[12] = 10;

	setpointsRARM[13] = 10;

	if (numberCircles == 1)
		setpointsRARM[14] = 180; // Middle 2,3,4,5:10; 1:180;
	else
		setpointsRARM[14] = 10;

	if (numberCircles == 1 || numberCircles == 2)
		setpointsRARM[15] = 180; // Powers 3,4,5:180; 1,2:10;
	else
		setpointsRARM[15] = 10;

	posRARM->setPositionMode();
	posRARM->positionMove(setpointsRARM.data());
}

void Controller::leftHandCountingToFive(int numberCircles, CTime::time_point t0) {
	auto t1 = CTime::now();
	fsec fs = t1 - t0;

	setpointsLARM[0] = -60 + 4 * sin(fs.count()/3);
	setpointsLARM[1] = 75 + 4 * cos(fs.count()/2);
	setpointsLARM[2] = 0;
	setpointsLARM[3] = 90;
	setpointsLARM[4] = 0;
	setpointsLARM[5] = 0;
	setpointsLARM[6] = 0;
	setpointsLARM[7] = 50;
	setpointsLARM[8] = 20;
	setpointsLARM[9] = 70;

	if (numberCircles ==5 )
		setpointsLARM[10] = 10; // THUMB 1-4:180, 5:10
	else
		setpointsLARM[10] = 180;

	setpointsLARM[11] = 10;

	if (numberCircles == 3 )
		setpointsLARM[12] = 180; // INDEX 1,2,4,5:10;  3: 180;
	else
		setpointsLARM[12] = 10;

	setpointsLARM[13] = 10;

	if (numberCircles == 1)
		setpointsLARM[14] = 180; // Middle 2,3,4,5:10; 1:180;
	else
		setpointsLARM[14] = 10;

	if (numberCircles == 1 || numberCircles == 2)
		setpointsLARM[15] = 180; // Powers 3,4,5:180; 1,2:10;
	else
		setpointsLARM[15] = 10;

	posLARM->setPositionMode();
	posLARM->positionMove(setpointsLARM.data());
}

void Controller::lookAround(CTime::time_point t0) {
	auto t1 = CTime::now();
	fsec fs = t1 - t0;

	setpointsHead[0] = 5 + 10 * sin(fs.count());
	setpointsHead[1] = 0;
	setpointsHead[2] = 0;
	setpointsHead[3] = 5 + 10 * cos(fs.count());
	setpointsHead[4] = 5 + 10 * cos(fs.count());
	setpointsHead[5] = 0;

	posHead->setPositionMode();
	posHead->positionMove(setpointsHead.data());
}

void Controller::resetLARM() {
    setpointsLARM[0] = 5;
    setpointsLARM[1] = 10;
    setpointsLARM[2] = 0;
    setpointsLARM[3] = 20;
    setpointsLARM[4] = 0;
    setpointsLARM[5] = 0;
    setpointsLARM[6] = 0;
    setpointsLARM[7] = 50;
    setpointsLARM[8] = 20;
    setpointsLARM[9] = 70;
    setpointsLARM[10] = 20;
    setpointsLARM[11] = 10;
    setpointsLARM[12] = 10;
    setpointsLARM[13] = 10;
    setpointsLARM[14] = 10;
    setpointsLARM[15] = 10;

    posLARM->setPositionMode();
    posLARM->positionMove(setpointsLARM.data());
}

void Controller::resetRARM() {
    setpointsRARM[0] = 5;
    setpointsRARM[1] = 10;
    setpointsRARM[2] = 0;
    setpointsRARM[3] = 20;
    setpointsRARM[4] = 0;
    setpointsRARM[5] = 0;
    setpointsRARM[6] = 0;
    setpointsRARM[7] = 50;
    setpointsRARM[8] = 20;
    setpointsRARM[9] = 70;
    setpointsRARM[10] = 20;
    setpointsRARM[11] = 10;
    setpointsRARM[12] = 10;
    setpointsRARM[13] = 10;
    setpointsRARM[14] = 10;
    setpointsRARM[15] = 10;

    posRARM->setPositionMode();
    posRARM->positionMove(setpointsRARM.data());
}

void Controller::resetHEAD() {
    setpointsHead[0] = 0;
    setpointsHead[1] = 0;
    setpointsHead[2] = 0;
    setpointsHead[3] = 0;
    setpointsHead[4] = 0;
    setpointsHead[5] = 0;

    posHead->setPositionMode();
    posHead->positionMove(setpointsHead.data());
}

void Controller::resetPos() {
    std::cout << "Resetting Left Arm" << std::endl;
    resetLARM();
    std::cout << "Resetting Right Arm" << std::endl;
    resetRARM();
    std::cout << "Resetting Head" << std::endl;
    resetHEAD();
    std::cout << "Reset Robot" << std::endl;
}

Point Controller::checkScreenFace() {
    ImageOf<PixelRgb> *imageRead = imagePort.read();  // read an image
    ImageOf<PixelRgb> yarpImage;
    yarpImage.resize(300, 200);
    yarpImage = *imageRead;
    IplImage *cv_image = cvCreateImage(cvSize(yarpImage.width(), yarpImage.height()), IPL_DEPTH_8U, 3);
    cvCvtColor((IplImage*)yarpImage.getIplImage(), cv_image, CV_RGB2BGR);
    Mat imgMat = cv::cvarrToMat(cv_image);

    imageOut.prepare() = yarpImage;
    imageOut.write();


    bool truth = false;
    std::vector<Rect> faces;
    Mat image_gray;

    cvtColor(imgMat, image_gray, CV_BGR2GRAY);
    //equalizeHist(image_gray, image_gray);

    //-- Detect faces
    face_cascade.detectMultiScale(image_gray, faces, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, Size(30, 30));
	Point center = Point(0);
    if (faces.size() > 0) {
        truth = true;
		center = Point(faces[0].x + faces[0].width*0.5, faces[0].y + faces[0].height*0.5);
		std::cout << "Center: " << center << std::endl;
    }

    //for (int i = 0; i < faces.size(); i++) {
		//aux = true;
		//std::cout << "Drawing a circle \n";
		//Point center(faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5);
		//ellipse(imgMat, center, Size(faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar(255, 0, 255), 4, 8, 0);

		//Mat faceROI = image_gray(faces[i]);
		//std::vector<Rect> eyes;
		/*
		//-- In each face, detect eyes
		eyes_cascade.detectMultiScale(faceROI, eyes, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, Size(30, 30));

		for (size_t j = 0; j < eyes.size(); j++)
		{
			Point center(faces[i].x + eyes[j].x + eyes[j].width*0.5, faces[i].y + eyes[j].y + eyes[j].height*0.5);
			int radius = cvRound((eyes[j].width + eyes[j].height)*0.25);
			circle(imgMat, center, radius, Scalar(255, 0, 0), 4, 8, 0);
		}*/
    //}

    //-- Show what you got

    //imshow(window_name, imgMat);
    //std::cout << truth;
    //return truth;
    return center;
}

int Controller::checkScreenLinear() {
    ImageOf<PixelRgb> *image = imagePort.read();  // read an image
    int conf = 0;
    if (image != NULL) { // check we actually got something
        float xMean = 0;
        float yMean = 0;
        int greenCount = 0;
        for (int x = 0; x < image->width(); x++) {
            for (int y = 0; y < image->height(); y++) {
                PixelRgb &pixel = image->pixel(x, y);
                if (pixel.g > 120 && pixel.b < 20 && pixel.r < 20) {
                    /* there's a greenish pixel at (x,y)! */
                    /* let's find the average location of these pixels */
                    // apply filter
                    xMean += x;
                    yMean += y;
                    greenCount++;

                    image->pixel(x, y).r = 0;
                    image->pixel(x, y).g = 200;
                    image->pixel(x, y).b = 0;
                } else {
                    image->pixel(x, y).r = 0;
                    image->pixel(x, y).g = 0;
                    image->pixel(x, y).b = 0;
                    //linear_filter();
                }
            }
        }
        imageOut.prepare() = *image;
        imageOut.write();

        if (greenCount > 0) {
            xMean /= greenCount;
            yMean /= greenCount;
        }
        double x;
        double y;
        if (greenCount > (image->width() / 20) * (image->height() / 20)) {
            conf = 1;
            x = xMean;
            y = yMean;
        } else {
            x = 0;
            y = 0;
            conf = 0;
        }
        x -= 320 / 2; // centre of screen
        y -= 240 / 2; // centre of screen
        // HEAD
        double vx = x * 0.1;
        double vy = -y * 0.1;

        // prepare command
        for (int i = 0; i < jntsHead; i++) {
            setpointsHead[i] = 0;
        }
        if (conf > 0.5) {
            setpointsHead[0] = 0;
			setpointsHead[1] = 0;
            setpointsHead[3] = vy;
            setpointsHead[4] = vx;
            velHead->setVelocityMode();
            velHead->velocityMove(setpointsHead.data());
        }
        std::cout << "vx: " << vx << "\tvy: " << vy << "\tconf: " << conf << std::endl;
    }
    return conf;
}

void Controller::linear_filter() {
    Mat imgMat = getImage();

    int i = 0;
    int c;
    Mat dst;
    int ddepth = -1;
    int delta = 0;
    Point anchor = Point(-1, -1);
    Mat kernel;
    int kernel_size;

    kernel_size = 3 + 2 * (i % 5);
    kernel = Mat::ones(kernel_size, kernel_size, CV_32F) / (float)(kernel_size*kernel_size);
    filter2D(imgMat, dst, ddepth, kernel, anchor, delta, BORDER_DEFAULT);
    filter2D(dst, dst, ddepth, kernel, anchor, delta, BORDER_DEFAULT);

    i++;

    IplImage* imageTest;
    imageTest = cvCreateImage(cvSize(dst.cols, dst.rows), 8, 3);

    IplImage ipltemp2 = dst;
    cvCopy(&ipltemp2, imageTest);

    ImageOf <PixelRgb> sendingBlur;
    cvConvertImage(imageTest, imageTest, CV_CVTIMG_SWAP_RB);
    sendingBlur.wrapIplImage(imageTest);

    imageOut.prepare() = sendingBlur;
    imageOut.write();
}

int Controller::checkCircles() {
	Mat another = getImage();
	bool detected = false;
	Mat src_gray;

	/// Convert it to gray
	cvtColor(another, src_gray, CV_BGR2GRAY);

	/// Reduce the noise so we avoid false circle detection
	GaussianBlur(src_gray, src_gray, Size(9, 9), 2, 2);

	vector<Vec3f> circles;

	/// Apply the Hough Transform to find the circles
	HoughCircles(src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows / 8, 140, 25, 0, 0);

	if (circles.size() > 0) {
		detected = true;
	}
	int i = circles.size();
	std::cout << " found " << i << " circles\n";
	return circles.size();
}

Mat Controller::getImage() {
	std::cout << "Enter getImage()" << std::endl;
	ImageOf<PixelRgb> *imageRead = imagePort.read();  // read an image
	ImageOf<PixelRgb> yarpImage;
	yarpImage.resize(300, 200);
	yarpImage = *imageRead;
	IplImage *cv_image = cvCreateImage(cvSize(yarpImage.width(), yarpImage.height()), IPL_DEPTH_8U, 3);
	cvCvtColor((IplImage*)yarpImage.getIplImage(), cv_image, CV_RGB2BGR);
	std::cout << "Exit getImage()" << std::endl;
	return cv::cvarrToMat(cv_image);
}

int Controller::init() {
    imagePort.open("/tutorial/image/in");  // give the port a name
    imageOut.open("/tutorial/target/out");
    facePort.open("/tutorial/face/in");

    Network::connect("/icubSim/cam/left", "/tutorial/image/in");
    Network::connect("/tutorial/target/out", "/view/linear");
    Network::connect("/icubSim/cam/right", " /view/right");
    Network::connect("/icubSim/cam/left", "/view/left");
    //Network::connect("/test/video", "/icubSim/texture/screen");
    //Network::connect( "/tutorial/face/in", "/icubSim/texture/screen");
    Network::connect( "/grabber", "/icubSim/texture/screen");


    optionsHead.put("device", "remote_controlboard");
    optionsHead.put("local", "/tutorial/motor/head");
    optionsHead.put("remote", "/icubSim/head");
    optionsRARM.put("device", "remote_controlboard");
    optionsRARM.put("local", "/tutorial/motor/RARM");
    optionsRARM.put("remote", "/icubSim/right_arm");
    optionsLARM.put("device", "remote_controlboard");
    optionsLARM.put("local", "/tutorial/motor/LARM");
    optionsLARM.put("remote", "/icubSim/left_arm");

    PolyDriver robotHead(optionsHead);
    if (!robotHead.isValid()) {
        printf("Cannot connect to robot head\n");
        return 1;
    }
    PolyDriver robotRARM(optionsRARM);
    if (!robotRARM.isValid()) {
        printf("Cannot connect to robot RARM\n");
        return 1;
    }
    PolyDriver robotLARM(optionsLARM);
    if (!robotLARM.isValid()) {
        printf("Cannot connect to robot LARM\n");
        return 1;
    }

    robotHead.view(posHead);
    robotHead.view(velHead);
    robotHead.view(encHead);
    robotRARM.view(posRARM);
    robotRARM.view(velRARM);
    robotRARM.view(encRARM);
    robotLARM.view(posLARM);
    robotLARM.view(velLARM);
    robotLARM.view(encLARM);

    if (posHead == NULL || velHead == NULL || encHead == NULL) {
        printf("Cannot get interface to robot head\n");
        robotHead.close();
        return 1;
    }
    if (posRARM == NULL || velRARM == NULL || encRARM == NULL) {
        printf("Cannot get interface to robot right ARM\n");
        robotRARM.close();
        return 1;
    }
    if (posLARM == NULL || velLARM == NULL || encLARM == NULL) {
        printf("Cannot get interface to robot left ARM\n");
        robotLARM.close();
        return 1;
    }

    jntsHead = 0;
    posHead->getAxes(&jntsHead);
    setpointsHead.resize(jntsHead);

    jntsRARM = 0;
    posRARM->getAxes(&jntsRARM);
    setpointsRARM.resize(jntsRARM);

    jntsLARM = 0;
    posLARM->getAxes(&jntsLARM);
    setpointsLARM.resize(jntsLARM);

    std::cout << "jntsHEAD: " << jntsHead << std::endl;
    std::cout << "jntsRARM: " << jntsRARM << std::endl;
    std::cout << "posHEAD: " << posHead << std::endl;
    std::cout << "posRARM: " << posRARM << std::endl;

    //Mat src = imread("Blank.jpg");

    if (!face_cascade.load(face_cascade_name)) { printf("--(!)Error loading\n"); return -1; };
    if (!eyes_cascade.load(eyes_cascade_name)) { printf("--(!)Error loading\n"); return -1; };

	/*
    IplImage* image2;
    image2 = cvCreateImage(cvSize(src.cols, src.rows), 8, 3);
    IplImage ipltemp = src;
    cvCopy(&ipltemp, image2);
    ImageOf <PixelRgb> sendingImage;
    sendingImage.wrapIplImage(image2);

    facePort.prepare() = sendingImage;
    facePort.write();
    */

    resetPos();
    return 0;
}

int main() {
    Controller *c = new Controller();
    int error = c->init();
    if (error != 0)
        return error;

    StateMachine *sm = new StateMachine(c);
    return 0;
}
