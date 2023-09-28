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

    final static int EXPONENT_MODIFIER = 2;

    final static double VX_WEIGHT = .8;
    final static double VY_WEIGHT = .8;
    final static double VRX_WEIGHT = .8;

    @Override
    public void init(HardwareMap hwMap) {
        drive = new MecanumDrive(hwMap, new Pose2d(0, 0, 0));
    }

    @Override
    public void loop(Gamepad gamepad) {
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad.left_stick_x * VX_WEIGHT,
                        -gamepad.left_stick_y * VY_WEIGHT
                ),
                -gamepad.right_stick_x * VRX_WEIGHT
        ));
    }

    public double poweredInput(double base) {
        if (EXPONENT_MODIFIER % 2 == 0) {
            return Math.pow(base, EXPONENT_MODIFIER) * Math.signum(base);
        } else {
            return Math.pow(base, EXPONENT_MODIFIER);
        }
    }
}
