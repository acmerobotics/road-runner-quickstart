package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Mechanism;

import java.util.ArrayList;
import java.util.Iterator;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class Scorer extends Mechanism {

    Arm arm;
    Intake intake;

    enum ScoringState {
        state1, state2, state3, state4
    }

    ScoringState activeScoringState = ScoringState.state1;

    @Override
    public void init(HardwareMap hwMap) {
        arm = new Arm();
        intake = new Intake();
        intake.init(hwMap);
        arm.init(hwMap);
    }

    @Override
    public void loop(Gamepad gamepad) {
        switch (activeScoringState) {
            case state1:

            case state2:

            case state3:

            case state4:

        }
    }


}

