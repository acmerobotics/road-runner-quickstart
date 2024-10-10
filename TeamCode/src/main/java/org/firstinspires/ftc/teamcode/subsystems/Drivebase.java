package org.firstinspires.ftc.teamcode.subsystems;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.gb.pinpoint.driver.GoBildaPinpointDriver;
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

    GoBildaPinpointDriver odometryController;

    @Override
    public void init(HardwareMap hwMap) {
        frontLeft = hwMap.get(DcMotorEx.class, ConfigurationInfo.leftFront.getDeviceName());
        backLeft = hwMap.get(DcMotorEx.class, ConfigurationInfo.leftBack.getDeviceName());
        frontRight = hwMap.get(DcMotorEx.class, ConfigurationInfo.rightFront.getDeviceName());
        backRight = hwMap.get(DcMotorEx.class, ConfigurationInfo.rightBack.getDeviceName());
        setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        odometryController = hwMap.get(GoBildaPinpointDriver.class, "ODO");
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
        //pass
    }

    public void drive(AIMPad gamepad) {
        double x = poweredInput(deadzonedStickInput(gamepad.getLeftStickX()));
        double y = poweredInput(deadzonedStickInput(-gamepad.getLeftStickX()));
        double rx = poweredInput(deadzonedStickInput(gamepad.getRightStickX()));
        double denominator = computeDenominator(x, y, rx);

        frontLeftPower = (y + x + rx) / denominator;
        backLeftPower = (y - x - rx) / denominator;
        frontRightPower = (y - x + rx) / denominator;
        backRightPower = (y + x - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    public double deadzonedStickInput(double input) {
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
    public double poweredInput(double base) {
        if (GamepadSettings.EXPONENT_MODIFIER % 2 == 0) {
            return Math.pow(base, GamepadSettings.EXPONENT_MODIFIER) * Math.signum(base);
        } else {
            return Math.pow(base, GamepadSettings.EXPONENT_MODIFIER);
        }
    }

    public double computeDenominator(double x, double y, double rx) {
        return Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
    }
}
