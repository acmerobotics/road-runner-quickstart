package org.firstinspires.ftc.teamcode.drive.opmode.visionCenterStage;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

// This file will probably be deleted later
public class auxillaryPipeline extends OpenCvPipeline {
    // A submat (submatrix) is a way to create a reference to a region of interest within an existing matrix (or image)
    Mat submat;

    @Override
    public void init(Mat firstFrame)
    {
        // This creates a submatrix of the firstFrame matrix
        // Represents the top-left 50x50 portion of the matrix
        // DO YOU SEE WHERE THIS COULD BE RELEVANT?? APRILTAGS?!?! WE SEE APRILTAG WE MAKE SUBMATRIX WE READ IT!!!
        submat = firstFrame.submat(0,50,0,50);
    }

    @Override
    public Mat processFrame(Mat input) // takes a parameter input of type Mat (representing a new frame)
    {
        // Returns the previously initialized submat
        return submat;

        // The docs say that any changes to 'input' will be reflected in the submat and vice-versa
        // If the image changes, then the submat will change too
        // If you make a change to the submat, then that region of the original image will be affected
        // It seems as though if we make a submat of an apriltag, team prop, etc. we could possibly change the color of that portion
        // of the image and that could help our robot recognize stuff like that
    }

}
