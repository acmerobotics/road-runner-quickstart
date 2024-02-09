package org.firstinspires.ftc.teamcode.drive.opmode.visionPowerPlay;
import androidx.annotation.NonNull;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class parkingZoneFinder extends OpenCvPipeline {

    public enum parkingZone {
        ZONE1,
        ZONE2,
        ZONE3,
        UNKNOWN
    }

    // Scalars to define color ranges of interest

    // (key , Bule , red)
    public final Scalar oneLower = new Scalar(0.0, 100.0, 0.0);
    public final Scalar oneUpper = new Scalar(255.0, 255.0, 100.0);

    public final Scalar twoLower = new Scalar(0.0, 150.0, 0.0);
    public final Scalar twoUpper = new Scalar(255.0, 255.0, 255.0);

    public final Scalar threeLower = new Scalar(0.0, 0.0, 140.0);
    public final Scalar threeUpper = new Scalar(255.0, 135.0, 255.0);

    private final Mat YcBcR = new Mat();
    private final Mat oneMat = new Mat();
    private final Mat twoMat = new Mat();
    private final Mat threeMat = new Mat();
    private Mat targetMat = new Mat();
    private double oneAvg, twoAvg, threeAvg;

    private final Mat output = new Mat();

    private final Rect targetArea = new Rect(565, 340, 210, 290);

    private int zoneNumber = -1;

    public Mat processFrame(@NonNull Mat input) {

        // We reduce our mat to only hold the pixels we are interested in
        targetMat = input.submat(targetArea);
        // We convert that mat to the YcBcR colorspace for easier masking
        Imgproc.cvtColor(targetMat, YcBcR, Imgproc.COLOR_RGB2YCrCb);

        // The YcBcR conversion is then split into three separate masks for each parking zone
        Core.inRange(YcBcR, oneLower, oneUpper, oneMat);
        Core.inRange(YcBcR, twoLower, twoUpper, twoMat);
        Core.inRange(YcBcR, threeLower, threeUpper, threeMat);

        // Now we see how many pixels made it through the masking process in each mask
        oneAvg = Core.mean(oneMat).val[0];
        twoAvg = Core.mean(twoMat).val[0];
        threeAvg = Core.mean(threeMat).val[0];

        // Depending on which mask has the most pixels, we select a parking zone
        if (oneAvg > twoAvg && oneAvg > threeAvg) {
            zoneNumber = 1;
        } else if (twoAvg > oneAvg && twoAvg > threeAvg) {
            zoneNumber = 2;
        } else if (threeAvg > oneAvg && threeAvg > twoAvg) {
            zoneNumber = 3;
        }

        // Copy our input to our output for easier debugging
        input.copyTo(output);

        // Display our target area rectangle on the camera stream to help aim the camera
        Imgproc.rectangle(output, targetArea, new Scalar(255.0, 0.0, 0.0), 2);

        // Write the current parking zone on the camera stream for easier debugging
        Imgproc.putText(output, "Zone #" + zoneNumber, new Point(25, 100), Imgproc.FONT_HERSHEY_SIMPLEX, 3.0, new Scalar(0.0, 255.0, 0.0));

        return output;
    }

    // This allows us to get the zone out from the finder pipeline
    // It may be a different setup using tensorflow but I can help at the meet
    public parkingZone getParkingZone() {
        if (oneAvg > twoAvg && oneAvg > threeAvg) {
            return parkingZone.ZONE1;
        } else if (twoAvg > oneAvg && twoAvg > threeAvg) {
            return parkingZone.ZONE2;
        } else if (threeAvg > oneAvg && threeAvg > twoAvg) {
            return parkingZone.ZONE3;
        } else {
            return parkingZone.UNKNOWN;
        }
    }
}

/*
Color ranges for each parking zone

1: (0, 128, 0) (255, 255, 90)
2: (0, 180, 0) (255, 255, 150)
3: (0, 43, 160) (255, 106, 185)
 */

