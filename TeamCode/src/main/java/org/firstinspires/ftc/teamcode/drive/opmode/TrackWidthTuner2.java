package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.LoggingUtil;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

@Config
@Autonomous(group = "drive")
public class TrackWidthTuner2 extends LinearOpMode {

    public static final File TRACK_WIDTH_DIR =
            new File(LoggingUtil.ROAD_RUNNER_FOLDER, "trackwidth");

    private static double power(long timeNanos) {
        return 0.5 * Math.sin(1e-9 * timeNanos);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        DcMotorEx leftFront, leftRear, rightRear, rightFront;
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        class Data {
            final List<Double> angVelTimes = new ArrayList<>();
            final List<Double> angVels = new ArrayList<>();
            final List<Double> encVelTimes = new ArrayList<>();
            final List<Double> encVels = new ArrayList<>();
        }

        Data data = new Data();

        long t0 = System.nanoTime();
        while (opModeIsActive()) {
            leftFront.setPower(-power(System.nanoTime() - t0));
            rightFront.setPower(power(System.nanoTime() - t0));
            leftRear.setPower(-power(System.nanoTime() - t0));
            rightRear.setPower(power(System.nanoTime() - t0));

            long t1 = System.nanoTime();
            data.encVels.add(
                    0.25 * (rightRear.getVelocity()) + rightFront.getVelocity()
                            - leftFront.getVelocity() - leftRear.getVelocity());
            long t2 = System.nanoTime();
            // TODO: adjust for SDK bug
            data.angVels.add((double) -imu.getAngularVelocity().xRotationRate);
            long t3 = System.nanoTime();

            data.encVelTimes.add(0.5e-9 * (t1 + t2));
            data.angVelTimes.add(0.5e-9 * (t2 + t3));
        }

        try {
            new ObjectMapper(new JsonFactory())
                    .writerWithDefaultPrettyPrinter()
                    .writeValue(new File(TRACK_WIDTH_DIR,
                                    System.currentTimeMillis() + ".json"),
                            data);
        } catch (IOException e) {
            RobotLog.setGlobalErrorMsg(new RuntimeException(e),
                    "unable to write track width data");
        }
    }
}
