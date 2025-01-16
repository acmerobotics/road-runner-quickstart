package org.firstinspires.ftc.teamcode.subsystems.v1;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.settings.ConfigurationInfo;

public class Outdated_Intake extends Mechanism {

    CRServo bristles;
    Servo leftHinge;
    Servo rightHinge;
    final double DOWN_HINGE_POSITION = 0.14;
    final double NEUTRAL_HINGE_POSITION = 0.35;
    final double UP_HINGE_POSITION = 0.76;

    ColorSensor leftCS;
    ColorSensor rightCS;

    int[] redBlockValues = {1400, 400, 150};
    int[] blueBlockValues = {150, 400, 800};
    int[] yellowBlockValues = {1200, 1200, 150};

    int[] currentLeftCS = {0, 0, 0};
    int[] currentRightCS = {0, 0, 0};

    enum HingeState {
        UP, NEUTRAL, DOWN, CUSTOM
    }

    private HingeState activeHingeState = HingeState.NEUTRAL;
    double hingeTargetPosition = UP_HINGE_POSITION;

    @Override
    public void init(HardwareMap hwMap) {
        bristles = hwMap.get(CRServo.class, ConfigurationInfo.bristles.getDeviceName());
        leftHinge = hwMap.get(Servo.class, ConfigurationInfo.leftHinge.getDeviceName());
        rightHinge = hwMap.get(Servo.class, ConfigurationInfo.rightHinge.getDeviceName());
        rightHinge.setDirection(Servo.Direction.REVERSE);

        leftCS = hwMap.get(ColorSensor.class, ConfigurationInfo.leftCS.getDeviceName());
        rightCS = hwMap.get(ColorSensor.class, ConfigurationInfo.rightCS.getDeviceName());
    }

    @Override
    public void loop(AIMPad aimpad) {
        switch(activeHingeState) {
            case DOWN:
                hingeDownState();
                break;
            case NEUTRAL:
                hingeNeutralState();
                break;
            case UP:
                hingeUpState();
                break;
            case CUSTOM:
                break;
        }
        updateCS();
        hingeToPosition(hingeTargetPosition);
    }

    public void setActiveHingeState(HingeState activeHingeState) {
        this.activeHingeState = activeHingeState;
    }

    public void setHingeStateCustom(double position) {
        setActiveHingeState(HingeState.CUSTOM);
        hingeTargetPosition = position;
    }

    public void hingeDownState() {
        hingeTargetPosition = DOWN_HINGE_POSITION;
    }
    public void hingeUpState() {
        hingeTargetPosition = UP_HINGE_POSITION;
    }
    public void hingeNeutralState() {
        hingeTargetPosition = NEUTRAL_HINGE_POSITION;
    }


    public void bristlesIn() {bristles.setPower(.4);}

    public void bristlesOut() {bristles.setPower(-.7);}

    public void bristlesOff(){
        bristles.setPower(0);
    }

    public void bristlesAtPower(double bristlesPower) {
        bristles.setPower(bristlesPower);
    }

    public void hingeToPosition(double hingePosition) {
        double clampedHingePosition = Math.max(DOWN_HINGE_POSITION, Math.min(hingePosition, UP_HINGE_POSITION));
        leftHinge.setPosition(clampedHingePosition);
        rightHinge.setPosition(clampedHingePosition);
    }

    private void updateCS() {
        currentLeftCS[0] = leftCS.red();
        currentLeftCS[1] = leftCS.green();
        currentLeftCS[2] = leftCS.blue();
        currentRightCS[0] = rightCS.red();
        currentRightCS[1] = rightCS.green();
        currentRightCS[2] = rightCS.blue();
    }

    private boolean matchesColor(int[] currentCS, int[] colorValues) {
        return currentCS[0] > colorValues[0] && currentCS[1] > colorValues[1] && currentCS[2] > colorValues[2];
    }

    public boolean isBlockRed() {
        return matchesColor(currentLeftCS, redBlockValues) || matchesColor(currentRightCS, redBlockValues);
    }

    public boolean isBlockBlue() {
        return matchesColor(currentLeftCS, blueBlockValues) || matchesColor(currentRightCS, blueBlockValues);
    }

    public boolean isBlockYellow() {
        return matchesColor(currentLeftCS, yellowBlockValues) || matchesColor(currentRightCS, yellowBlockValues);
    }

    /**
     * Detects the color of the block in front of the sensors.
     *
     * @return the detected block color as a String
     */
    public String getBlockColor() {
        if (isBlockYellow()) return "YELLOW";
        if (isBlockRed()) return "RED";
        if (isBlockBlue()) return "BLUE";
        return "NONE";
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Block Color", getBlockColor());
        telemetry.addData("Left Sensor Red", currentLeftCS[0]);
        telemetry.addData("Left Sensor Green", currentLeftCS[1]);
        telemetry.addData("Left Sensor Blue", currentLeftCS[2]);

        telemetry.addData("Right Sensor Red", currentRightCS[0]);
        telemetry.addData("Right Sensor Green", currentRightCS[1]);
        telemetry.addData("Right Sensor Blue", currentRightCS[2]);
    }

    public void systemsCheck(AIMPad aimpad, Telemetry telemetry) {
        if (aimpad.isAPressed()) {
            setActiveHingeState(HingeState.UP);
        } else if (aimpad.isBPressed()) {
            setActiveHingeState(HingeState.DOWN);
        } else if (aimpad.isXPressed()) {
            setActiveHingeState(HingeState.NEUTRAL);
        } else if (aimpad.isYHeld()) {
            setHingeStateCustom(aimpad.getLeftStickX());
        }
        bristlesAtPower(aimpad.getRightStickY());
        loop(aimpad);
        telemetry(telemetry);
    }

}
