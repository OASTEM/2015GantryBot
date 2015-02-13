package org.oastem.frc.sensor;

//import org.bytedeco.javacpp.Loader;
//import org.bytedeco.javacpp.opencv_objdetect;
/*import org.bytedeco.javacv.*;
import org.bytedeco.javacv.FrameGrabber.Exception;

import static org.bytedeco.javacpp.opencv_core.*;
//import static org.bytedeco.javacpp.opencv_imgproc.*;

public class ImageProcessing {
	private OpenCVFrameGrabber grabber;
	private IplImage grabbedImage;
	public void checkWebcam() {
		try {
			grabber.start();
			grabbedImage = grabber.grab();
			System.out.println("LEEBROOOOOOOOOOOON");
		} catch (Exception e) {
			System.out.println("Oh yeah oh yeah webcam not working oh no");
		}
	}
	
	public ImageProcessing() {
		grabber = new OpenCVFrameGrabber("");
	}
}

/** 		Loader.load(opencv_objdetect.class);
OpenCVFrameGrabber grabber = new OpenCVFrameGrabber(0);
grabber.start();
IplImage grabbedImage = grabber.grab();
cvFlip(grabbedImage, grabbedImage, 1);
int width = grabbedImage.width();
int height = grabbedImage.height();
IplImage grayImage = IplImage.create( width, height, IPL_DEPTH_8U, 1 );
CvMemStorage storage = CvMemStorage.create();
CanvasFrame canvas = new CanvasFrame("oh yeah");
while (canvas.isVisible() && (grabbedImage = grabber.grab()) != null ) {
    cvClearMemStorage( storage );
    cvCvtColor( grabbedImage, grayImage, CV_BGR2GRAY );
    cvThreshold( grayImage, grayImage, 128, 255, CV_THRESH_BINARY );
    CvSeq contour = new CvSeq(null);
    cvFindContours( grayImage.clone(), storage, contour, Loader.sizeof( CvContour.class ), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE );
    while (contour != null && !contour.isNull()) {
        if (contour.elem_size() > 0) {
            CvSeq points = cvApproxPoly(contour, Loader.sizeof( CvContour.class ), storage, CV_POLY_APPROX_DP, cvContourPerimeter(contour) * 0.02, 0);
            cvDrawContours(grabbedImage, points, CvScalar.BLUE, CvScalar.BLUE, -1, 1, CV_AA);
        	System.out.println("Webcam is working");
        }
        contour = contour.h_next();
    }

    canvas.showImage( grabbedImage );
}
grabber.stop();
**/