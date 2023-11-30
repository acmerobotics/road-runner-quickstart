
package org.firstinspires.ftc.teamcode.testing.structureOptions.inheritanceStructure;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name="Example linear OpMode", group="Linear Opmode")

public class exampleLinearTeleOp extends LinearOpMode {

    private BNO055IMU imu = null;
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        motor1 = hardwareMap.get(DcMotor.class, "Motor 0");
        motor2 = hardwareMap.get(DcMotor.class, "Motor 1");

        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addLine("THERE IS TEXT!");
        telemetry.update();



        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu.initialize(parameters);

        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.

        waitForStart();

        while (opModeIsActive()) {
            motor1.setPower(gamepad1.left_stick_y);
            motor2.setPower(gamepad1.right_stick_y);
            telemetry.addLine("Motor1 pos: " + motor1.getCurrentPosition());
            telemetry.addLine("Motor2 pos: " + motor2.getCurrentPosition());
            telemetry.addLine("imu: " + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);


            telemetry.update();
        }
    }
}
