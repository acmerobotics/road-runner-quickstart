package org.firstinspires.ftc.teamcode.opMode.protoType;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.MeccRobot;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

@TeleOp
public class distanceTster extends LinearOpMode {
    public MeccRobot robot = new MeccRobot();
    DistanceSensor distance;
    @Override
    public void runOpMode() throws InterruptedException {
        distance = hardwareMap.get(DistanceSensor.class, "distance");
        Rev2mDistanceSensor temp = (Rev2mDistanceSensor)distance;
        waitForStart();
        while(opModeIsActive()) {
            robot.run(gamepad1);
            telemetry.addData("distance: ", temp.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
