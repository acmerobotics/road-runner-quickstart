package org.firstinspires.ftc.teamcode.subsystems;

import com.aimrobotics.aimlib.control.PIDController;
import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.gb.pinpoint.driver.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.gb.pinpoint.driver.Pose2D;
import org.firstinspires.ftc.teamcode.settings.ConfigurationInfo;
import org.firstinspires.ftc.teamcode.settings.GamepadSettings;
import org.firstinspires.ftc.teamcode.util.InputModification;

import java.util.Locale;

public class Drivebase extends Mechanism {

    private DcMotorEx frontLeft;
    private DcMotorEx backLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backRight;

    private double frontLeftPower;
    private double backLeftPower;
    private double frontRightPower;
    private double backRightPower;

    private GoBildaPinpointDriver odometryController;

    private Pose2D targetPose;
    private Pose2D startingPose;

    public enum DriveMode {
        MANUAL, TO_POSITION
    }

    private DriveMode activeDriveMode = DriveMode.MANUAL;

    PIDController axialPID;
    PIDController lateralPID;
    PIDController turnPID;

    private final static double XY_PROXIMITY_THRESHOLD = 1;
    private final static double HEADING_PROXIMITY_THRESHOLD = 8;

    public Drivebase(Pose2D startingPosition) {
        startingPose = startingPosition;
    }

    @Override
    public void init(HardwareMap hwMap) {
        frontLeft = hwMap.get(DcMotorEx.class, ConfigurationInfo.leftFront.getDeviceName());
        backLeft = hwMap.get(DcMotorEx.class, ConfigurationInfo.leftBack.getDeviceName());
        frontRight = hwMap.get(DcMotorEx.class, ConfigurationInfo.rightFront.getDeviceName());
        backRight = hwMap.get(DcMotorEx.class, ConfigurationInfo.rightBack.getDeviceName());
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        odometryController = hwMap.get(GoBildaPinpointDriver.class, "ODO");
        odometryController.setOffsets(-44.45, 38.1);
        odometryController.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odometryController.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odometryController.resetPosAndIMU();
        odometryController.setPosition(startingPose);
        targetPose = odometryController.getPosition();

        double axial_ku = 0.13;
        double axial_tu = 1.4;

        double lateral_ku = 0.13;
        double lateral_tu = 1.4;

        double axialKP = axial_ku * 0.6;
        double axialKI = (1.2 * axial_ku)/ axial_tu;
        double axialKD = 0.075 * axial_ku * axial_tu;
        double axialDerivLowPass = 0;
        double axialIntMax = 0;

        double lateralKP = lateral_ku * 0.6;
        double lateralKI = (1.2 * lateral_ku)/ lateral_tu;
        double lateralKD = 0.075 * lateral_ku * lateral_tu;
        double lateralDerivLowPass = 0;
        double lateralIntMax = 0;

        double turnKP = 0.008; // 0.005
        double turnKI = 0;
        double turnKD = 0;
        double turnDerivLowPass = 0;
        double turnIntMax = 0;

        axialPID = new PIDController(axialKP, axialKI, axialKD, axialDerivLowPass, axialIntMax);
        lateralPID = new PIDController(lateralKP, lateralKI, lateralKD, lateralDerivLowPass, lateralIntMax);
        turnPID = new PIDController(turnKP, turnKI, turnKD, turnDerivLowPass, turnIntMax);
    }

    public void setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior behavior) {
        frontLeft.setZeroPowerBehavior(behavior);
        backLeft.setZeroPowerBehavior(behavior);
        frontRight.setZeroPowerBehavior(behavior);
        backRight.setZeroPowerBehavior(behavior);
    }

    public void setMode(DcMotorEx.RunMode mode) {
        frontLeft.setMode(mode);
        backLeft.setMode(mode);
        frontRight.setMode(mode);
        backRight.setMode(mode);
    }

    @Override
    public void loop(AIMPad gamepad1) {
        switch (activeDriveMode) {
            case MANUAL:
                manualDrive(gamepad1);
                break;
            case TO_POSITION:
                updateAutoNavigation();
                break;
        }
        odometryController.bulkUpdate();
    }

    public void setActiveDriveMode(DriveMode driveMode) {
        activeDriveMode = driveMode;
    }

    private void manualDrive(AIMPad gamepad) {
        double y = InputModification.poweredInput(deadzonedStickInput(-gamepad.getLeftStickY()), GamepadSettings.EXPONENT_MODIFIER);
        double x = InputModification.poweredInput(deadzonedStickInput(gamepad.getLeftStickX()), GamepadSettings.EXPONENT_MODIFIER);
        double rx = InputModification.poweredInput(deadzonedStickInput(gamepad.getRightStickX()), GamepadSettings.EXPONENT_MODIFIER);
        moveDrivebase(y, x, rx);
    }

    private void moveDrivebase(double y, double x, double rx) {
        double denominator = computeDenominator(y, x, rx);
        backRightPower = (y + x + rx) / denominator;
        frontRightPower = (y - x + rx) / denominator;
        backLeftPower = (y - x - rx) / denominator;
        frontLeftPower = (y + x - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    private double deadzonedStickInput(double input) {
        if (Math.abs(input) > GamepadSettings.GP1_STICK_DEADZONE) {
            return input;
        } else {
            return 0;
        }
    }

    private double computeDenominator(double y, double x, double rx) {
        return Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        Pose2D pos = odometryController.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);
    }

    public void systemsCheck(AIMPad gamepad, Telemetry telemetry) {
        loop(gamepad);
    }

    public void setTargetPose(Pose2D targetPose) {
        this.targetPose = targetPose;
    }

    public void updateAutoNavigation() {
        double axialPower = axialPID.calculate(targetPose.getX(DistanceUnit.INCH), odometryController.getPosition().getX(DistanceUnit.INCH));
        double lateralPower = lateralPID.calculate(targetPose.getY(DistanceUnit.INCH), odometryController.getPosition().getY(DistanceUnit.INCH));
        double turnPower = turnPID.calculate(targetPose.getHeading(AngleUnit.DEGREES), odometryController.getPosition().getHeading(AngleUnit.DEGREES));
        moveDrivebase(axialPower, lateralPower, -turnPower);
    }

    public boolean isAtTargetPosition() {
        boolean isAtY = Math.abs(odometryController.getPosition().getY(DistanceUnit.INCH) - targetPose.getY(DistanceUnit.INCH)) < XY_PROXIMITY_THRESHOLD;
        boolean isAtX = Math.abs(odometryController.getPosition().getX(DistanceUnit.INCH) - targetPose.getX(DistanceUnit.INCH)) < XY_PROXIMITY_THRESHOLD;
        boolean isAtHeading = Math.abs(odometryController.getPosition().getHeading(AngleUnit.DEGREES) - targetPose.getHeading(AngleUnit.DEGREES)) < HEADING_PROXIMITY_THRESHOLD;

        return isAtY && isAtX && isAtHeading;
    }
}
