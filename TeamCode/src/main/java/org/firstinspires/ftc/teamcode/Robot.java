package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

public class Robot {

    public Lift lift;
    public Arm arm;
    public Wrist wrist;
    public Claw claw;
    public MecanumDrive drive;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.lift = new Lift(hardwareMap, telemetry);
        this.arm = new Arm(hardwareMap, telemetry);
        this.wrist = new Wrist(hardwareMap, telemetry);
        this.claw = new Claw(hardwareMap, telemetry);
        this.drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));


    }

    //TODO: husky lens/opencv centering script, combinbed lift, arm, wrist action
}
