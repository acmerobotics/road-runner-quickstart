package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

public class DriveSubsystem extends SubsystemBase {
    private final MecanumDrive drive;
    private final Telemetry telemetry;

    public DriveSubsystem(final HardwareMap hardwareMap, @NonNull Telemetry telemetry) {
        drive = new MecanumDrive(hardwareMap);
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        telemetry.addLine("Drivetrain")
                .addData("Left Front", drive.getWheelVelocities().get(0))
                .addData("Left Rear", drive.getWheelVelocities().get(1))
                .addData("Right Front", drive.getWheelVelocities().get(2))
                .addData("Right Rear", drive.getWheelVelocities().get(3));

    }

    public void setWeightedDrivePower(@NonNull Pose2d drivePower) {
        drive.setWeightedDrivePower(drivePower);
    }
}
