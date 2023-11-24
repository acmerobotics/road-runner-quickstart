package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name="Robot: 22312Tele", group="Robot")
public class RobotTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        Constants constants = new Constants();
        Map<String, String> stateMap = new HashMap<String, String>() {{ }};
        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, stateMap);
        stateMap.put(robot.intake.INTAKE_SYSTEM_NAME, robot.intake.INTAKE_IDLE_STATE);
        stateMap.put(robot.hopper.HOPPER_SYSTEM_NAME, robot.hopper.HOPPER_NO_PIXELS);
        stateMap.put(robot.fulcrum.FULCRUM_SYSTEM_NAME, robot.fulcrum.FULCRUM_DOWN);
        stateMap.put(constants.NUMBER_OF_PIXELS, constants.PIXEL_PICKUP_2_PIXELS);
        stateMap.put(robot.arm.ARM_SYSTEM_NAME,robot.arm.ARM_IDLE_STATE);
        stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_GROUND_STATE);
        stateMap.put(robot.drawbridge.DRAWBRIDGE_SYSTEM_NAME,robot.drawbridge.DRAWBRIDGE_UP_STATE);
        waitForStart();

        while(!isStopRequested()){
            telemetry.addLine("IN Tele");
            if(gamepad1.dpad_up){
                stateMap.put(robot.arm.ARM_SYSTEM_NAME, robot.arm.ARM_DEPOSIT_STATE);
            } else if(gamepad1.dpad_down){
                stateMap.put(robot.arm.ARM_SYSTEM_NAME, robot.arm.ARM_IDLE_STATE);
            }
            if(gamepad1.a){
                stateMap.put(robot.drawbridge.DRAWBRIDGE_SYSTEM_NAME, robot.drawbridge.DRAWBRIDGE_DOWN_STATE);

            }
            if(gamepad1.b){
                stateMap.put(robot.drawbridge.DRAWBRIDGE_SYSTEM_NAME, robot.drawbridge.DRAWBRIDGE_UP_STATE);
            }
//                stateMap.put(robot.constants.PIXEL_CYCLE, robot.constants.PIXEL_CYCLE_STATE_IN_PROGRESS);
//            if(gamepad1.dpad_down){
//                stateMap.put(robot.fulcrum.FULCRUM_SYSTEM_NAME, robot.fulcrum.FULCRUM_UP);
//            }
            robot.drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x * 0.75));

            robot.drive.updatePoseEstimate();
            robot.updateSystems();
            telemetry.update();
        }
    }
}
