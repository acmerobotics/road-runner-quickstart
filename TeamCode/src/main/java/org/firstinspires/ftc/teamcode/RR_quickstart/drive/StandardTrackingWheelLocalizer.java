package org.firstinspires.ftc.teamcode.RR_quickstart.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.RR_quickstart.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = (35 / 2.0) / 25.4; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    int previous_left_pos = 0;
    int previous_right_pos = 0;
    int previous_middle_pos = 0;
    final int REJECT_CUTOFF = (int) TICKS_PER_REV * 10;

    public static double LATERAL_DISTANCE = 10; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -0.84; // in; offset of the lateral wheel

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "FrontLeft"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "FrontRight"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BackLeft"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        frontEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {


        int leftPos = leftEncoder.getCurrentPosition();
        int rightPos = rightEncoder.getCurrentPosition();
        int midPos = frontEncoder.getCurrentPosition();
//
//        if (Math.abs(leftPos - previous_left_pos) > REJECT_CUTOFF) {
//            int temp = leftPos;
//            leftPos = previous_left_pos;
//            previous_left_pos = temp;
//        }
//        if (Math.abs(rightPos - previous_right_pos) > REJECT_CUTOFF) {
//            int temp = rightPos;
//            rightPos = previous_right_pos;
//            previous_right_pos = temp;
//        }
//        if (Math.abs(midPos - previous_middle_pos) > REJECT_CUTOFF) {
//            int temp = midPos;
//            midPos = previous_middle_pos;
//            previous_middle_pos = temp;
//        }

        return Arrays.asList(
                encoderTicksToInches(leftPos),
                encoderTicksToInches(rightPos),
                encoderTicksToInches(-midPos)
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()),
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()),
                encoderTicksToInches(-frontEncoder.getCorrectedVelocity())
        );
    }
}
