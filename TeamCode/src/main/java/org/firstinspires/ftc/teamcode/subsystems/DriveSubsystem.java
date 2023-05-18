package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

public class DriveSubsystem extends SubsystemBase {
    private final MecanumDrive drive;

    public DriveSubsystem(final HardwareMap hardwareMap) {
        drive = new MecanumDrive(hardwareMap);
    }

    public void setWeightedDrivePower(@NonNull Pose2d drivePower) {
        drive.setWeightedDrivePower(drivePower);
    }
}
