package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.Wrist;

@TeleOp(name="Teleop Modular", group="Robot")
//@Disabled
@Config
public class TeleOpModular extends LinearOpMode {

    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int)Arm.ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor = 0;


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
        Arm arm = new Arm(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Lift lift = new Lift(hardwareMap);

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

            drive.moveRobotFieldCentric(y, x, rx);

            y = squaredInputWithSign(y);
//          x = squaredInputWithSign(x)
            if (gamepad1.options) {
                drive.resetYaw();
            }
            /* Here we handle the three buttons that have direct control of the intake speed.
            These control the continuous rotation servo that pulls elements into the robot,
            If the user presses A, it sets the intake power to the final variable that
            holds the speed we want to collect at.
            If the user presses X, it sets the servo to Off.
            And if the user presses B it reveres the servo to spit out the element.*/

            /* TECH TIP: If Else statements:
            We're using an else if statement on "gamepad1.x" and "gamepad1.b" just in case
            multiple buttons are pressed at the same time. If the driver presses both "a" and "x"
            at the same time. "a" will win over and the intake will turn on. If we just had
            three if statements, then it will set the intake servo's power to multiple speeds in
            one cycle. Which can cause strange behavior. */

            if (gamepad1.a) {
                intake.collect();
            }
            else if (gamepad1.x) {
                intake.stop();
            }
            else if (gamepad1.b) {
                intake.deposit();
            }

            /* Here we implement a set of if else statements to set our arm to different scoring positions.
            We check to see if a specific button is pressed, and then move the arm (and sometimes
            intake and wrist) to match. For example, if we click the right bumper we want the robot
            to start collecting. So it moves the armPosition to the ARM_COLLECT position,
            it folds out the wrist to make sure it is in the correct orientation to intake, and it
            turns the intake on to the COLLECT mode.*/

            if(gamepad1.right_bumper){
                /* This is the intaking/collecting arm position */
                armPosition = Arm.ARM_COLLECT;
                wrist.foldOut();
                intake.collect();
            }

            else if (gamepad1.left_bumper){
                    /* This is about 20° up from the collecting position to clear the barrier
                    Note here that we don't set the wrist position or the intake power when we
                    select this "mode", this means that the intake and wrist will continue what
                    they were doing before we clicked left bumper. */
                armPosition = Arm.ARM_CLEAR_BARRIER;
            }

            else if (gamepad1.y){
                /* This is the correct height to score the sample in the LOW BASKET */
                armPosition = Arm.ARM_SCORE_SAMPLE_IN_LOW;
            }

            else if (gamepad1.dpad_left) {
                    /* This turns off the intake, folds in the wrist, and moves the arm
                    back to folded inside the robot. This is also the starting configuration */
                armPosition = Arm.ARM_COLLAPSED_INTO_ROBOT;
                intake.stop();
                wrist.foldIn();
            }

            else if (gamepad1.dpad_right){
                /* This is the correct height to score SPECIMEN on the HIGH CHAMBER */
                armPosition = Arm.ARM_SCORE_SPECIMEN;
                wrist.foldIn();
            }

            else if (gamepad1.dpad_up){
                /* This sets the arm to vertical to hook onto the LOW RUNG for hanging */
                armPosition = Arm.ARM_ATTACH_HANGING_HOOK;
                intake.stop();
                wrist.foldIn();
            }

            else if (gamepad1.dpad_down){
                /* this moves the arm down to lift the robot up once it has been hooked */
                armPosition = Arm.ARM_WINCH_ROBOT;
                intake.stop();
                wrist.foldIn();
            }

            /* Here we create a "fudge factor" for the arm position.
            This allows you to adjust (or "fudge") the arm position slightly with the gamepad triggers.
            We want the left trigger to move the arm up, and right trigger to move the arm down.
            So we add the right trigger's variable to the inverse of the left trigger. If you pull
            both triggers an equal amount, they cancel and leave the arm at zero. But if one is larger
            than the other, it "wins out". This variable is then multiplied by our FUDGE_FACTOR.
            The FUDGE_FACTOR is the number of degrees that we can adjust the arm by with this function. */

            armPositionFudgeFactor = Arm.FUDGE_FACTOR * (gamepad1.right_trigger + (-gamepad1.left_trigger));


            /* Here we set the target position of our arm to match the variable that was selected
            by the driver.
            We also set the target velocity (speed) the motor runs at, and use setMode to run it.*/
            arm.motor.setTargetPosition((int) (armPosition + armPositionFudgeFactor));
            arm.motor.setVelocity(2500);
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
            liftPosition += liftPower;

            if (gamepad2.right_bumper){
                liftPosition += 2800 * cycletime;
            }
            else if (gamepad2.left_bumper){
                liftPosition -= 2800 * cycletime;
            }
            
            if (gamepad2.a) {
                wrist.foldOut();
            } else if (gamepad2.b) {
                wrist.foldIn();
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

            lift.motor.setTargetPosition((int) (liftPosition));

            lift.motor.setVelocity(1300);
            lift.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


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
            telemetry.addData("lift target" , lift.motor.getTargetPosition());
            telemetry.addData("lift position", lift.motor.getCurrentPosition());
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
