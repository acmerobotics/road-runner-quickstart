package org.firstinspires.ftc.teamcode.drive.opmode.visionCenterStage;

// Both included packages
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


// https://github.com/OpenFTC/EasyOpenCV/blob/master/doc/user_docs/pipelines_overview.md
// Very simple pipeline
public class cameraPipeline extends OpenCvPipeline // All pipelines must extend OpenCvPipeline
{
    // Mats are arrays that are used to store data of images
    // If you read an image using OpenCV it will read to a Mat object
    // Take note that this is declared as an instance variable and not a local variable
    Mat grey = new Mat();

    @Override
    // Converts the image to grey
    public Mat processFrame(Mat input)
    {
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGB2YCrCb);
        return grey; // Returns the output
    }
}
/*
Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGB2YCrCb);

This line does image processing
CvtColor converts an image from one colorspace to another, such as converting from RGB to Grey or to YCrCb(which is what they used last year)

SPECIFICS
    input = The image we want to convert
    grey = The output image where the result of the conversion will be stored (in a Mat object)
    Imgproc.COLOR_RBG2yCrCb specifies that it will be converting the image from RGB to YCrCb

 */

// WE NEED TO INITIALIZE THE CAMERA
// My problem is that I don't know where these things are supposed to be initialized in the code
// Like where do we initialize the connected stuff like cameras, servos, etc.

// https://github.com/OpenFTC/EasyOpenCV/blob/master/doc/user_docs/camera_initialization_overview.md

// There is a file in FTCRobotController called UtilityCameraFrameCapture which I think may be important later on
