package org.firstinspires.ftc.teamcode.subsystems.multiaxisarm;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MultiAxisArm extends Mechanism {

    public Hand hand;
    public Wrist wrist;
    public Elbow elbow;

    @Override
    public void init(HardwareMap hwMap) {
        hand = new Hand();
        wrist = new Wrist();
        elbow = new Elbow();

        hand.init(hwMap);
        wrist.init(hwMap);
        elbow.init(hwMap);
    }

    @Override
    public void loop(AIMPad aimpad, AIMPad aimpad2) {
        hand.loop(aimpad);
        wrist.loop(aimpad);
        elbow.loop(aimpad);
    }

    public void toggleSpecimen() {
        wrist.toggleSpecimen();
        elbow.toggleSpecimen();
    }

    public void resetOpen() {
        hand.open();
        wrist.flexNeutral();
        wrist.rotateCenter();
        elbow.middle();
    }

    public void specimenPickup() {
        hand.open();
        wrist.flexNeutral();
        wrist.rotateCenter();
        elbow.forward();
    }

    public void neutral() {
        hand.open();
        wrist.flexNeutral();
        wrist.rotateCenter();
        elbow.middle();
    }

    public void neutralClosed() {
        hand.close();
        wrist.flexNeutral();
        wrist.rotateCenter();
        elbow.middle();
    }

    public void upClosed() {
        hand.close();
        wrist.flexScore();
        wrist.rotateCenter();
        elbow.score();
    }

    public void hang() {
        hand.close();
        wrist.flexNeutral();
        wrist.rotateCenter();
        elbow.hang();
    }

    public void scoreNew() {
        hand.close();
        wrist.flexScoreNew();
        wrist.rotateCenter();
        elbow.score();
    }

    public void custom(double handPosition, double wristFlexPosition, double wristRotatePosition, double elbowPosition) {
        hand.custom(handPosition);
        wrist.flexCustom(wristFlexPosition);
        wrist.rotateCustom(wristRotatePosition);
        elbow.custom(elbowPosition);
    }
}
