package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.Mechanism;

public class Drivebase extends Mechanism {

    MecanumDrive drive;

    @Override
    public void init(HardwareMap hwMap) {
        drive = new MecanumDrive(hwMap, new Pose2d(0, 0, 0));
    }

    @Override
    public void loop(Gamepad gamepad) {
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        poweredInput(stickValWithDeadzone(-gamepad.left_stick_y) * GamepadSettings.VY_WEIGHT),
                        poweredInput(stickValWithDeadzone(-gamepad.left_stick_x) * GamepadSettings.VX_WEIGHT)
                ),
                poweredInput(stickValWithDeadzone(-gamepad.right_stick_x) * GamepadSettings.VRX_WEIGHT)
        ));
    }

    public double poweredInput(double base) {
        if (GamepadSettings.EXPONENT_MODIFIER % 2 == 0) {
            return Math.pow(base, GamepadSettings.EXPONENT_MODIFIER) * Math.signum(base);
        } else {
            return Math.pow(base, GamepadSettings.EXPONENT_MODIFIER);
        }
    }

    public double stickValWithDeadzone(double stickVal) {
        if (Math.abs(stickVal) > GamepadSettings.GP1_STICK_DEADZONE) {
            return stickVal;
        } else {
            return 0;
        }
    }
}
