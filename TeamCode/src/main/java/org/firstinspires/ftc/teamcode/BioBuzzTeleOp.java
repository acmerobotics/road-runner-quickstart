package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (group="RoadRunner TeleOp")
public class BioBuzzTeleOp extends LinearOpMode {
    private MecanumDrive drive;
    private HiveVision hiveVision;

    private Turret turret;
    private Intake intake;

    private Action driveAction = null;

    private final Pose2d shootPose1 = new Pose2d(40, -12, Math.toRadians(180));

    private final Pose2d shootPose2 = new Pose2d(-40, -12, Math.toRadians(0));

    private final Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

    @Override
    public void runOpMode() {
        initializeRobot();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

            // --------------------------------
            // Sensors / localization
            // --------------------------------

            updateSensors();

            hiveVision.update();
            updateVisionPose();

            // --------------------------------
            // Driver input
            // --------------------------------

            updateButtons();

            // --------------------------------
            // Drivetrain
            // --------------------------------

            updateDrive(packet);

            // --------------------------------
            // Other subsystems
            // --------------------------------

            updateTurret();
            updateIntake();

            // --------------------------------
            // Telemetry
            // --------------------------------

            updateTelemetry(packet);

            telemetry.update();
        }
    }

    private void initializeRobot() {
        drive = new MecanumDrive(
                hardwareMap,
                startPose
        );

        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);

        hiveVision = new HiveVision(hardwareMap);

        // Any other initialization
    }

    private void updateSensors() {
        // Read/update sensors that aren't handled elsewhere
    }

    // ============================================================
    // DRIVETRAIN
    // ============================================================

    private void updateDrive(TelemetryPacket packet) {
        if (driveAction != null) {
            updateAutomaticDrive(packet);
        } else {
            updateManualDrive();
        }
    }

    private void updateManualDrive() {
        // Driver-controlled mecanum movement
        //
        // gamepad1 left stick
        // gamepad1 right stick
        //
        // Send movement command to drive
    }

    private void updateAutomaticDrive(TelemetryPacket packet) {
        boolean running = driveAction.run(packet);

        if (!running) {
            driveAction = null;
        }
    }

    // ============================================================
    // AUTOMATIC POSITIONING
    // ============================================================

    private void startShootPath() {
        Pose2d currentPose = drive.localizer.getPose();

        Pose2d targetPose = chooseShootPose();

        driveAction = buildShootPath(
                currentPose,
                targetPose
        );
    }

    private Action buildShootPath(
            Pose2d startPose,
            Pose2d targetPose
    ) {
        // Build a Road Runner action here.
        //
        // Example conceptually:
        //
        // return drive.actionBuilder(startPose)
        //     .strafeTo(...)
        //     .turnTo(...)
        //     .build();

        return null;
    }

    private Pose2d chooseShootPose() {
        // Decide which shooting position to use.
        //
        // Could eventually use:
        // - current robot position
        // - alliance
        // - closest shooting position
        // - game state
        // - driver selection

        return shootPose1;
    }

    private void cancelShootPath() {
        driveAction = null;

        stopDrive();
    }

    private void stopDrive() {
        // Stop drivetrain motors
    }

    // ============================================================
    // VISION / HIVE
    // ============================================================

    private void updateVisionPose() {
        Pose2d visionPose = hiveVision.getNewRobotPose();

        if (visionPose == null) {
            return;
        }

        /*
         * Initial/simple strategy:
         *
         * Only correct the Road Runner pose while the
         * drivetrain is under driver control.
         *
         * Don't overwrite the pose while an RR action
         * is actively following a trajectory.
         */

        if (driveAction == null) {
            drive.localizer.setPose(visionPose);
        }
    }

    // ============================================================
    // BUTTONS
    // ============================================================

    private void updateButtons() {
        boolean aPressed = gamepad1.aWasPressed();

        boolean bPressed = gamepad1.bWasPressed();

        if (aPressed && driveAction == null) {
            startShootPath();
        }

        if (bPressed && driveAction != null) {
            cancelShootPath();
        }
    }

    // ============================================================
    // TURRET
    // ============================================================

    private void updateTurret() {
        // Turret logic runs EVERY loop.
        //
        // Can use HIVE vision information here.
    }

    // ============================================================
    // INTAKE
    // ============================================================

    private void updateIntake() {
        // Intake logic runs EVERY loop.
    }

    // ============================================================
    // TELEMETRY
    // ============================================================

    private void updateTelemetry(TelemetryPacket packet) {
        telemetry.addData(
                "Pose",
                drive.localizer.getPose()
        );

        telemetry.addData(
                "Driving Mode",
                driveAction == null
                        ? "MANUAL"
                        : "AUTOMATIC"
        );

        telemetry.addData(
                "HIVE Pose",
                hiveVision.getLatestPose()
        );

        // Add RR/dashboard telemetry to packet here
    }
}