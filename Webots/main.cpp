
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Image.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h> 
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace yarp::sig;
using namespace yarp::os;

int main(int argc, char *argv[])
{
	Property p;
	Network yarp; 
	BufferedPort<ImageOf<PixelRgb> > imagePort; // port for reading images
	imagePort.open("/imageProc/image/in");  // give the port a name

	while (1) { // repeat forever
		ImageOf<PixelRgb> *image = imagePort.read();  // read an image

		if (image != NULL) { // check we actually got something
			printf("We got an image of size %dx%d\n", image->width(), image->height());
		}
	}
	return 0;
}
