package org.firstinspires.ftc.teamcode.robot;

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

        stateMap.put(constants.NUMBER_OF_PIXELS, constants.PIXEL_PICKUP_2_PIXELS);

        waitForStart();

        while(!isStopRequested()){
            telemetry.addLine("IN Tele");
            if(gamepad1.right_trigger > 0.5){
                stateMap.put(constants.PIXEL_CYCLE, constants.PIXEL_CYCLE_STATE_IN_PROGRESS);
//                robot.intake.intakeMotor.setPower(gamepad1.right_trigger);
            }
            robot.updateSystems();
            telemetry.update();
        }
    }
}
