package org.firstinspires.ftc.teamcode.TeleOp;


import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.robotv2.Armv2;
import org.firstinspires.ftc.teamcode.mechanisms.robotv2.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.robotv2.LeftActuator;
import org.firstinspires.ftc.teamcode.mechanisms.robotv2.Liftparantap;
import org.firstinspires.ftc.teamcode.mechanisms.robotv2.RightActuator;
import org.firstinspires.ftc.teamcode.mechanisms.robotv2.Wristv2;


@TeleOp(name="Teleop Modular V3", group="Robot")
@Disabled
public class TeleOpModular_V3 extends LinearOpMode {

    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int)Armv2.ARM_REST_POSITION;
    double armPositionFudgeFactor = 0;

    public static double ARM_SPECIMEN_SCORE_POSITION = 200;
    public static double LIFT_SCORE_POSITION = 650;

    public static int RETURN_SLIDES_POSTION = 1000;


    public static int RETURN_ACTUATOR_POSTION = 0;

    public static int LIFT_HANG_POSITION = -1150;

    public  static int LIFT_HANG_SLIDES_POSITION = -3200;


    static final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;

    final double LIFT_COLLAPSED = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_LOW_BASKET = 0 * LIFT_TICKS_PER_MM;
    public static double LIFT_SCORING_IN_HIGH_BASKET = 510 * LIFT_TICKS_PER_MM * 1.3;

    double liftPosition = LIFT_COLLAPSED;


    double cycletime = 0;
    double looptime = 0;
    double oldtime = 0;

    double armLiftComp = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Armv2 arm = new Armv2(hardwareMap);
        Wristv2 wrist = new Wristv2(hardwareMap);
        Liftparantap liftparantap = new Liftparantap(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        LeftActuator leftActuator = new LeftActuator(hardwareMap);
        RightActuator rightActuator = new RightActuator(hardwareMap);

        claw.clawClose();
        wrist.WristFoldIn();
        // temporary reset to run teleop only.
        arm.reset();
        armPosition = ARM_SPECIMEN_SCORE_POSITION;

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();


        /* Wait for the game driver to press play */
        waitForStart();

        while (opModeIsActive()) {

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            y = squaredInputWithSign(y);
            x = squaredInputWithSign(x);
            rx = squaredInputWithSign(rx);

            drive.moveRobotFieldCentric(y, x, rx);

            if (gamepad1.options) {
                drive.resetYaw();
            }

//            if(gamepad1.dpad_left){
//                //leftActuator.motor.setTargetPosition(LeftActuator.ACTUATOR_UP);
//                telemetry.addData("button pressed", "yes");
//
//                //leftActuator.motor.setTargetPosition(0);
//                //leftActuator.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                //leftActuator.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                leftActuator.motor.setTargetPosition(LIFT_HANG_POSITION);
//                leftActuator.motor.setVelocity(2000);
//                leftActuator.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                rightActuator.motor.setTargetPosition(LIFT_HANG_POSITION);
//                rightActuator.motor.setVelocity(2000);
//                rightActuator.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                //liftPosition = 1000;
//
//                liftparantap.motor.setTargetPosition(LIFT_HANG_SLIDES_POSITION);
//                liftparantap.motor.setVelocity(2000);
//                liftparantap.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//
//            } else if (gamepad1.dpad_right){
//                leftActuator.motor.setTargetPosition(RETURN_ACTUATOR_POSTION);
//                leftActuator.motor.setVelocity(2000);
//                leftActuator.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                rightActuator.motor.setTargetPosition(RETURN_ACTUATOR_POSTION);
//                rightActuator.motor.setVelocity(2000);
//                rightActuator.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                liftparantap.motor.setTargetPosition(RETURN_SLIDES_POSTION);
//                liftparantap.motor.setVelocity(2000);
//                liftparantap.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }


//            if (gamepad1.dpad_up){
//                claw.clawClose();
//                wrist.WristVertical();
//                armPosition = ARM_SPECIMEN_SCORE_POSITION;
//                liftPosition = LIFT_SCORE_POSITION;
//            } else if (gamepad1.dpad_down) {
//                claw.clawOpen();
//                armPosition = ARM_SPECIMEN_SCORE_POSITION;
//                liftPosition = LIFT_COLLAPSED;
//                wrist.WristDown();
//            }
//
            if (gamepad1.a) {
                claw.clawClose();
            }

            else if (gamepad1.b) {
                claw.clawOpen();
            }

            if(gamepad1.x){
                wrist.WristFoldOut();

            } else if (gamepad1.y) {
                wrist.WristFoldIn();
            }

            if (gamepad1.dpad_down){
                armPosition = Armv2.ARM_PICKUP_GROUND_SAMPLE;
            }


            /* Here we implement a set of if else statements to set our arm to different scoring positions.
            We check to see if a specific button is pressed, and then move the arm (and sometimes
            intake and wrist) to match. For example, if we click the right bumper we want the robot
            to start collecting. So it moves the armPosition to the ARM_COLLECT position,
            it folds out the wrist to make sure it is in the correct orientation to intake, and it
            turns the intake on to the COLLECT mode.*/
//
//            if(gamepad2.right_bumper){
//                /* This is the intaking/collecting arm position */
//                armPosition = Armv2.ARM_COLLECT;
//                liftPosition = 800;
//
//
//            }
//
//            else if (gamepad2.left_bumper) {
//                    /* This is about 20° up from the collecting position to clear the barrier
//                    Note here that we don't set the wrist position or the intake power when we
//                    select this "mode", this means that the intake and wrist will continue what
//                    they were doing before we clicked left bumper. */
//                armPosition = Armv2.ARM_CLEAR_BARRIER;
//                liftPosition = 800;
//
//            }
//
//            else if (gamepad2.y){
//                /* This is the correct height to score the sample in the LOW BASKET */
//                armPosition = Armv2.ARM_SCORE_SAMPLE_IN_LOW;
//            }
//
//            else if (gamepad2.dpad_left) {
//                    /* This turns off the intake, folds in the wrist, and moves the arm
//                    back to folded inside the robot. This is also the starting configuration */
//                armPosition = Armv2.ARM_COLLAPSED_INTO_ROBOT;
//                liftPosition = 0;
//
//            }
//
//            else if (gamepad2.dpad_right){
//                // score sample in high basket
//                armPosition = Armv2.ARM_SCORE_SAMPLE_IN_HIGH;
//                liftPosition = 2800;
//
//            }
//
//            else if (gamepad2.dpad_up){
//                /* This sets the arm to vertical to hook onto the LOW RUNG for hanging */
//                armPosition = Armv2.ARM_ATTACH_HANGING_HOOK;
//
//            }
//
//            else if (gamepad2.dpad_down){
//                /* this moves the arm down to lift the robot up once it has been hooked */
//                armPosition = Armv2.ARM_WINCH_ROBOT;
//
//            }

            /* Here we create a "fudge factor" for the arm position.
            This allows you to adjust (or "fudge") the arm position slightly with the gamepad triggers.
            We want the left trigger to move the arm up, and right trigger to move the arm down.
            So we add the right trigger's variable to the inverse of the left trigger. If you pull
            both triggers an equal amount, they cancel and leave the arm at zero. But if one is larger
            than the other, it "wins out". This variable is then multiplied by our FUDGE_FACTOR.
            The FUDGE_FACTOR is the number of degrees that we can adjust the arm by with this function. */

            armPositionFudgeFactor = Armv2.FUDGE_FACTOR * (gamepad1.right_trigger + (-gamepad1.left_trigger));


            /* Here we set the target position of our arm to match the variable that was selected
            by the driver.
            We also set the target velocity (speed) the motor runs at, and use setMode to run it.*/
            arm.motor.setTargetPosition((int) (armPosition + armPositionFudgeFactor));
            arm.motor.setVelocity(500);
            arm.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            /* TECH TIP: Encoders, integers, and doubles
            Encoders report when the motor has moved a specified angle. They send out pulses which
            only occur at specific intervals (see our ARM_TICKS_PER_DEGREE). This means that the
            position our arm is currently at can be expressed as a whole number of encoder "ticks".
            The encoder will never report a partial number of ticks. So we can store the position in
            an integer (or int).
            A lot of the variables we use in FTC are doubles. These can capture fractions of whole
            numbers. Which is great when we want our arm to move to 122.5°, or we want to set our
            servo power to 0.5.

            setTargetPosition is expecting a number of encoder ticks to drive to. Since encoder
            ticks are always whole numbers, it expects an int. But we want to think about our
            arm position in degrees. And we'd like to be able to set it to fractions of a degree.
            So we make our arm positions Doubles. This allows us to precisely multiply together
            armPosition and our armPositionFudgeFactor. But once we're done multiplying these
            variables. We can decide which exact encoder tick we want our motor to go to. We do
            this by "typecasting" our double, into an int. This takes our fractional double and
            rounds it to the nearest whole number.
            */



            /* Here we set the lift position based on the driver input.
            This is a.... weird, way to set the position of a "closed loop" device. The lift is run
            with encoders. So it knows exactly where it is, and there's a limit to how far in and
            out it should run. Normally with mechanisms like this we just tell it to run to an exact
            position. This works a lot like our arm. Where we click a button and it goes to a position, then stops.
            But the drivers wanted more "open loop" controls. So we want the lift to keep extending for
            as long as we hold the bumpers, and when we let go of the bumper, stop where it is at.
            This allows the driver to manually set the position, and not have to have a bunch of different
            options for how far out it goes. But it also lets us enforce the end stops for the slide
            in software. So that the motor can't run past it's endstops and stall.
            We have our liftPosition variable, which we increment or decrement for every cycle (every
            time our main robot code runs) that we're holding the button. Now since every cycle can take
            a different amount of time to complete, and we want the lift to move at a constant speed,
            we measure how long each cycle takes with the cycletime variable. Then multiply the
            speed we want the lift to run at (in mm/sec) by the cycletime variable. There's no way
            that our lift can move 2800mm in one cycle, but since each cycle is only a fraction of a second,
            we are only incrementing it a small amount each cycle.
             */

            double liftPower = (gamepad2.right_trigger - gamepad2.left_trigger);
            liftPosition += liftPower * 300;

            if (gamepad2.right_bumper){
                liftPosition += 2800 * cycletime;
            }
            else if (gamepad2.left_bumper){
                liftPosition -= 2800 * cycletime;
            }


            /*here we check to see if the lift is trying to go higher than the maximum extension.
             *if it is, we set the variable to the max.
             */
            if (liftPosition > LIFT_SCORING_IN_HIGH_BASKET){
                liftPosition = LIFT_SCORING_IN_HIGH_BASKET;
            }
            //same as above, we see if the lift is trying to go below 0, and if it is, we set it to 0.
            if (liftPosition < 0){
                liftPosition = 0;
            }

           // liftparantap.motor.setTargetPosition((int) (liftPosition));

            //liftparantap.motor.setVelocity(5000);
           // liftparantap.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            /* Check to see if our arm is over the current limit, and report via telemetry. */
            if (arm.motor.isOverCurrent()){
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
            }

            /* This is how we check our loop time. We create three variables:
            looptime is the current time when we hit this part of the code
            cycletime is the amount of time in seconds our current loop took
            oldtime is the time in seconds that the previous loop started at

            we find cycletime by just subtracting the old time from the current time.
            For example, lets say it is 12:01.1, and then a loop goes by and it's 12:01.2.
            We can take the current time (12:01.2) and subtract the oldtime (12:01.1) and we're left
            with just the difference, 0.1 seconds.

             */
            looptime = getRuntime();
            cycletime = looptime-oldtime;
            oldtime = looptime;

            /* send telemetry to the driver of the arm's current position and target position */
            telemetry.addData("wrist servo", wrist.wrist.getPosition());
            telemetry.addData("armTarget: ", arm.motor.getTargetPosition());
            telemetry.addData("arm Encoder: ", arm.motor.getCurrentPosition());
            telemetry.addData("lift target" , liftparantap.motor.getTargetPosition());
            telemetry.addData("lift position", liftparantap.motor.getCurrentPosition());
            telemetry.update();

        }

    }

    private double squaredInputWithSign(double input) {
        double output = input * input;
        if (input<0){
            output = -output;
        }
        return output;
    }
}
