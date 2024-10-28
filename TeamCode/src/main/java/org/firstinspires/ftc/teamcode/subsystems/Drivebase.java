package org.firstinspires.ftc.teamcode.subsystems;

import com.aimrobotics.aimlib.control.PIDController;
import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.gb.pinpoint.driver.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.gb.pinpoint.driver.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.settings.ConfigurationInfo;
import org.firstinspires.ftc.teamcode.subsystems.settings.GamepadSettings;

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

    private enum DriveMode {
        MANUAL, TO_POSITION
    }

    private DriveMode activeDriveMode = DriveMode.MANUAL;

    PIDController axialPID;
    PIDController lateralPID;
    PIDController turnPID;

    private final static double XY_PROXIMITY_THRESHOLD = 5;
    private final static double HEADING_PROXIMITY_THRESHOLD = 1;

    @Override
    public void init(HardwareMap hwMap) {
        frontLeft = hwMap.get(DcMotorEx.class, ConfigurationInfo.leftFront.getDeviceName());
        backLeft = hwMap.get(DcMotorEx.class, ConfigurationInfo.leftBack.getDeviceName());
        frontRight = hwMap.get(DcMotorEx.class, ConfigurationInfo.rightFront.getDeviceName());
        backRight = hwMap.get(DcMotorEx.class, ConfigurationInfo.rightBack.getDeviceName());
        setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        odometryController = hwMap.get(GoBildaPinpointDriver.class, "ODO");
        targetPose = odometryController.getPosition();

        double axialKP = 0;
        double axialKI = 0;
        double axialKD = 0;
        double axialDerivLowPass = 0;
        double axialIntMax = 0;

        double lateralKP = 0;
        double lateralKI = 0;
        double lateralKD = 0;
        double lateralDerivLowPass = 0;
        double lateralIntMax = 0;

        double turnKP = 0;
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
    }

    public void setActiveDriveMode(DriveMode driveMode) {
        activeDriveMode = driveMode;
    }

    private void manualDrive(AIMPad gamepad) {
        double y = poweredInput(deadzonedStickInput(-gamepad.getLeftStickY()));
        double x = poweredInput(deadzonedStickInput(gamepad.getLeftStickX()));
        double rx = poweredInput(deadzonedStickInput(gamepad.getRightStickX()));
        moveDrivebase(y, x, rx);
    }

    private void moveDrivebase(double y, double x, double rx) {
        double denominator = computeDenominator(y, x, rx);
        frontLeftPower = (y - x + rx) / denominator;
        backLeftPower = (y + x + rx) / denominator;
        frontRightPower = (y + x - rx) / denominator;
        backRightPower = (y - x - rx) / denominator;

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

    /**
     * Returns the powered input
     * @param base base input
     * @return base to the EXPONENT_MODIFIER power
     */
    private double poweredInput(double base) {
        if (GamepadSettings.EXPONENT_MODIFIER % 2 == 0) {
            return Math.pow(base, GamepadSettings.EXPONENT_MODIFIER) * Math.signum(base);
        } else {
            return Math.pow(base, GamepadSettings.EXPONENT_MODIFIER);
        }
    }

    private double computeDenominator(double y, double x, double rx) {
        return Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
    }

    public void setTargetPose(Pose2D targetPose) {
        this.targetPose = targetPose;
    }

    public void updateAutoNavigation() {
        double yPower = axialPID.calculate(targetPose.getY(DistanceUnit.INCH), odometryController.getPosY());
        double xPower = lateralPID.calculate(targetPose.getX(DistanceUnit.INCH), odometryController.getPosX());
        double rxPower = turnPID.calculate(targetPose.getHeading(AngleUnit.DEGREES), odometryController.getHeading());
        moveDrivebase(yPower, xPower, rxPower);
    }

    public boolean isAtTargetPosition() {
        boolean isAtY = Math.abs(odometryController.getPosition().getY(DistanceUnit.INCH) - targetPose.getY(DistanceUnit.INCH)) < XY_PROXIMITY_THRESHOLD;
        boolean isAtX = Math.abs(odometryController.getPosition().getX(DistanceUnit.INCH) - targetPose.getX(DistanceUnit.INCH)) < XY_PROXIMITY_THRESHOLD;
        boolean isAtHeading = Math.abs(odometryController.getPosition().getHeading(AngleUnit.DEGREES) - targetPose.getHeading(AngleUnit.DEGREES)) < HEADING_PROXIMITY_THRESHOLD;

        return isAtY && isAtX && isAtHeading;
    }

    public void moveToPosition(Pose2D targetPose) {
        setActiveDriveMode(DriveMode.TO_POSITION);
        setTargetPose(targetPose);
        while (!isAtTargetPosition()) {
            updateAutoNavigation();
        }
    }
}
