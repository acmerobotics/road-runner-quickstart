package org.firstinspires.ftc.teamcode.OldCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "UltimateWobbleDrop (Blocks to Java)", group = "")
public class UltimateWobbleDrop extends LinearOpMode {

  private Servo wobbleDropper;
  private DcMotor rightback;
  private DcMotor rightfront;
  private DcMotor leftfront;
  private DcMotor leftback;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    double driveTime;
    double motorPower;
    ElapsedTime elapsedTime;
    ElapsedTime wobbleTime;
    double wobbleTimeLimit;
    ElapsedTime pauseTimer;

    wobbleDropper = hardwareMap.get(Servo.class, "wobbleDropper");
    rightback = hardwareMap.get(DcMotor.class, "rightback");
    rightfront = hardwareMap.get(DcMotor.class, "rightfront");
    leftfront = hardwareMap.get(DcMotor.class, "leftfront");
    leftback = hardwareMap.get(DcMotor.class, "leftback");

    waitForStart();
    if (opModeIsActive()) {
      // wobbleDropper is the wobbleDropper Servo
      wobbleDropper.setPosition(0);
      // driveTime is the amount of time spent running forwards
      //Voltage: 14V drivetime: 1.4
      //Voltage: 12V driveTime: 1.7
      //Voltage: 10V driveTime: 1.9
      driveTime = 0.65;
      // motorPower is the power of the drive motors
      motorPower = 1.0;
      // How long to let the wobbleDropper drop
      wobbleTimeLimit = driveTime + 4;
      elapsedTime = new ElapsedTime();
      while (opModeIsActive()) {
        telemetry.addData("Time in Seconds:", elapsedTime.seconds());
        // When two motors are opposite each other, they turn in opposite directions.
        rightback.setDirection(DcMotorSimple.Direction.REVERSE);
        rightfront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftfront.setPower(motorPower);
        leftback.setPower(motorPower);
        rightfront.setPower(motorPower);
        rightback.setPower(motorPower);
        if (elapsedTime.seconds() >= driveTime) {
          leftfront.setPower(0);
          leftback.setPower(0);
          rightfront.setPower(0);
          rightback.setPower(0);
          break;
        }
        telemetry.update();
      }
      telemetry.addData("Time in Seconds:", elapsedTime.seconds());
      pauseTimer = new ElapsedTime();
      while (pauseTimer.seconds() < 1) {
        telemetry.addData("Time in Seconds:", pauseTimer.milliseconds());
      }
      // WobbleDropper
      wobbleDropper.setDirection(Servo.Direction.FORWARD);
      wobbleTime = new ElapsedTime();
      while (wobbleTime.seconds() < wobbleTimeLimit) {
        wobbleDropper.setPosition(1);
      }
      telemetry.addData("Time in Seconds:", wobbleTime.seconds());
    }
  }
}
