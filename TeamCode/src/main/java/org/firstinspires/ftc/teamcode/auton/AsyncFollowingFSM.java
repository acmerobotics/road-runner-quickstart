package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.oldCode.AutoCommon;
import org.firstinspires.ftc.teamcode.oldCode.RobotHardware;

/**
 * This opmode explains how you follow multiple trajectories in succession, asynchronously. This
 * allows you to run your own logic beside the drive.update() command. This enables one to run
 * their own loops in the background such as a PID controller for a lift. We can also continuously
 * write our pose to PoseStorage.
 * <p>
 * The use of a State enum and a currentState field constitutes a "finite state machine."
 * You should understand the basics of what a state machine is prior to reading this opmode. A good
 * explanation can be found here:
 * https://www.youtube.com/watch?v=Pu7PMN5NGkQ (A finite state machine introduction tailored to FTC)
 * or here:
 * https://gm0.org/en/stable/docs/software/finite-state-machines.html (gm0's article on FSM's)
 * <p>
 * You can expand upon the FSM concept and take advantage of command based programming, subsystems,
 * state charts (for cyclical and strongly enforced states), etc. There is still a lot to do
 * to supercharge your code. This can be much cleaner by abstracting many of these things. This
 * opmode only serves as an initial starting point.
 */
@Autonomous(group = "advanced")
public class AsyncFollowingFSM extends AutoCommon {

    public AsyncFollowingFSM(HardwareMap hardwareMap) {
        super.runOpMode();
    }
//    @Override
//    public void runOpMode() {
//
//        super.runOpMode();
//
////        Lift autoLift = new Lift(hardwareMap);
////        Lift.Turret autoTurret = new Lift.Turret(hardwareMap);
////        RobotHardware robot = null;
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        waitForStart();
//
//        if (isStopRequested()) return;

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum AutoState {
        TRAJECTORY_1,   // First, follow a splineTo() trajectory
        TRAJECTORY_2,   // Then, follow a lineTo() trajectory
        TURN_1,         // Then we want to do a point turn
        TRAJECTORY_3,   // Then, we follow another lineTo() trajectory
        WAIT_1,         // Then we're gonna wait a second
        TURN_2,         // Finally, we're gonna turn again
        IDLE            // Our bot will enter the IDLE state when done
    }

    enum LiftStates {
        HIGH_JUNCTION,
        MIDDLE_JUNCTION,
        LOW_JUNCTION,
        INTAKE
    }

    // We define the current state we're on
    // Default to IDLE
    AutoState pathCurrentState = AutoState.IDLE;
    LiftStates liftCurrentState = LiftStates.INTAKE;

    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
    Pose2d startPose = new Pose2d(34.5, -61.5, Math.toRadians(270));

    @Override
    public void runOpMode() {
//        super.runOpMode();
//        // Initialize our lift
////        Lift lift = new Lift(hardwareMap);
//        AsyncFollowingFSM sync = new AsyncFollowingFSM(hardwareMap);
//
//        // Initialize SampleMecanumDrive
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//
//        // Set inital pose
//
//        drive.setPoseEstimate(startPose);
//
//        Trajectory traj = drive.trajectoryBuilder(startPose)
//
//                .lineToConstantHeading(new Vector2d(34.5, -10),
//                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//
//                .build();
//
//        // Second trajectory
//        // Ensure that we call trajectory1.end() as the start for this one
//        Trajectory trajectory2 = drive.trajectoryBuilder(traj.end())
//                .lineTo(new Vector2d(45, 0))
//                .build();
//
//        // Define the angle to turn at
//        double turnAngle1 = Math.toRadians(-270);
//
//        // Third trajectory
//        // We have to define a new end pose because we can't just call trajectory2.end()
//        // Since there was a point turn before that
//        // So we just take the pose from trajectory2.end(), add the previous turn angle to it
//        Pose2d newLastPose = trajectory2.end().plus(new Pose2d(0, 0, turnAngle1));
//        Trajectory trajectory3 = drive.trajectoryBuilder(newLastPose)
//                .lineToConstantHeading(new Vector2d(-15, 0))
//                .build();
//
//        // Define a 1.5 second wait time
//        double waitTime1 = 1.5;
//        ElapsedTime waitTimer1 = new ElapsedTime();
//
//        // Define the angle for turn 2
//        double turnAngle2 = Math.toRadians(720);
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        // Set the current state to TRAJECTORY_1, our first step
//        // Then have it follow that trajectory
//        // Make sure you use the async version of the commands
//        // Otherwise it will be blocking and pause the program here until the trajectory finishes
//        pathCurrentState = AutoState.TRAJECTORY_1;
//        liftCurrentState = LiftStates.HIGH_JUNCTION;
//        drive.followTrajectoryAsync(traj);
//
//        while (opModeIsActive() && !isStopRequested()) {
//            // Our state machine logic
//            // You can have multiple switch statements running together for multiple state machines
//            // in parallel. This is the basic idea for subsystems and commands.
//
//            switch (liftCurrentState) {
//                case HIGH_JUNCTION:
//                    robot.lift.motorLiftR.setTargetPosition(1000);
//                    robot.lift.motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    robot.lift.motorLiftR.setPower(1);
//            }
//            break;
//            // We essentially define the flow of the state machine through this switch statement
//            switch (pathCurrentState) {
//                case TRAJECTORY_1:
//
//                    // Check if the drive class isn't busy
//                    // `isBusy() == true` while it's following the trajectory
//                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
//                    // We move on to the next state
//                    // Make sure we use the async follow function
//                    if (!drive.isBusy()) {
//                        pathCurrentState = AutoState.TRAJECTORY_2;
//                        drive.followTrajectoryAsync(trajectory2);
//                    }
//                    break;
////                case TRAJECTORY_2:
////                    // Check if the drive class is busy following the trajectory
////                    // Move on to the next state, TURN_1, once finished
////                    if (!drive.isBusy()) {
////                        pathCurrentState = AutoState.TURN_1;
////                        drive.turnAsync(turnAngle1);
////                    }
////                    break;
////                case TURN_1:
////                    // Check if the drive class is busy turning
////                    // If not, move onto the next state, TRAJECTORY_3, once finished
////                    if (!drive.isBusy()) {
////                        pathCurrentState = AutoState.TRAJECTORY_3;
////                        drive.followTrajectoryAsync(trajectory3);
////                    }
////                    break;
////                case TRAJECTORY_3:
////                    // Check if the drive class is busy following the trajectory
////                    // If not, move onto the next state, WAIT_1
////                    if (!drive.isBusy()) {
////                        pathCurrentState = AutoState.WAIT_1;
////
////                        // Start the wait timer once we switch to the next state
////                        // This is so we can track how long we've been in the WAIT_1 state
////                        waitTimer1.reset();
////                    }
////                    break;
////                case WAIT_1:
////                    // Check if the timer has exceeded the specified wait time
////                    // If so, move on to the TURN_2 state
////                    if (waitTimer1.seconds() >= waitTime1) {
////                        pathCurrentState = AutoState.TURN_2;
////                        drive.turnAsync(turnAngle2);
////                    }
////                    break;
////                case TURN_2:
////                    // Check if the drive class is busy turning
////                    // If not, move onto the next state, IDLE
////                    // We are done with the program
////                    if (!drive.isBusy()) {
////                        pathCurrentState = AutoState.IDLE;
////                    }
////                    break;
////                case IDLE:
////                    // Do nothing in IDLE
////                    // currentState does not change once in IDLE
////                    // This concludes the autonomous program
////                    break;
//            }
//
//            // Anything outside of the switch statement will run independent of the currentState
//
//            // We update drive continuously in the background, regardless of state
//            drive.update();
//            // We update our lift PID continuously in the background, regardless of state
////            lift.update();
//
//            // Read pose
//            Pose2d poseEstimate = drive.getPoseEstimate();
//
//
//            // Print pose to telemetry
//            telemetry.addData("x", poseEstimate.getX());
//            telemetry.addData("y", poseEstimate.getY());
//            telemetry.addData("heading", poseEstimate.getHeading());
//            telemetry.update();
//        }
//    }
//
////    // Assume we have a hardware class called lift
////    // Lift uses a PID controller to maintain its height
////    // Thus, update() must be called in a loop
////    class Lift {
////        public Lift(HardwareMap hardwareMap) {
//////            if (AsyncFO)
////
////        }
////
////
////        // Beep boop this is the the constructor for the lift
////        // Assume this sets up the lift hardware
////
////
////        public void update() {
////            // Beep boop this is the lift update function
////            // Assume this runs some PID controller for the lift
////        }
////    }
    }
}