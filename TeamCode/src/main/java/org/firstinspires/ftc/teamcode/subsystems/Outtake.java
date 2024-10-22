package org.firstinspires.ftc.teamcode.subsystems;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.settings.ConfigurationInfo;

public class Outtake extends Mechanism {

    Servo leftArmHinge;
    Servo rightArmHinge;
    Servo leftBucketHinge;
    Servo rightBucketHinge;

    final double ARM_HINGE_IN_POSITION = 0; //arm hinges to intake position
    final double ARM_HINGE_OUT_POSITION = 0; //arm hinges to outake position
    final double BUCKET_HINGE_IN_POSITION = 0; //bucket hinges to intake position
    final double BUCKET_HINGE_OUT_POSITION = 0; //bucket hinges to outake position

    enum ArmState {
        ARMIN, ARMOUT, CUSTOM
    }
    ArmState activeArmState = ArmState.ARMIN;
    double armTargetPosition = ARM_HINGE_OUT_POSITION;


    enum BucketState {
        BUCKETIN, BUCKETOUT, CUSTOM
    }
    BucketState activeBucketState = BucketState.BUCKETIN;
    double bucketTargetPosition = BUCKET_HINGE_IN_POSITION;


    @Override
    public void init(HardwareMap hwMap) {
        leftArmHinge = hwMap.get(Servo.class, ConfigurationInfo.leftArmHinge.getDeviceName());
        rightArmHinge = hwMap.get(Servo.class, ConfigurationInfo.rightArmHinge.getDeviceName());
        leftBucketHinge = hwMap.get(Servo.class, ConfigurationInfo.leftBucketHinge.getDeviceName());
        rightBucketHinge = hwMap.get(Servo.class, ConfigurationInfo.rightBucketHinge.getDeviceName());

    }

    @Override
    public void loop(AIMPad aimpad) {
        switch(activeArmState) {
            case ARMIN:
                armHingeIn();
                break;
            case ARMOUT:
                armHingeOut();
                break;
            case CUSTOM:
                break;
        }

        switch(activeBucketState) {
            case BUCKETIN:
                bucketHingeIn();
                break;
            case BUCKETOUT:
                bucketHingeOut();
                break;
            case CUSTOM:
                break;
        }
        armToPosition(armTargetPosition);
        bucketToPosition(bucketTargetPosition);
    }

    public void setActiveArmState(ArmState activeArmState) {
        this.activeArmState = activeArmState;
    }
    public void setArmStateCustom(double armPosition) {
        setActiveArmState(ArmState.CUSTOM);
        armTargetPosition = armPosition;
    }


    public void setActiveBucketState(BucketState activeBucketState) {
        this.activeBucketState = activeBucketState;
    }
    public void setBucketStateCustom(double bucketPosition) {
        setActiveBucketState(BucketState.CUSTOM);
        bucketTargetPosition = bucketPosition;
    }


    /**
     * puts arm hinge to intake position
     */
    public void armHingeIn() {
        armToPosition(ARM_HINGE_IN_POSITION);

    }

    /**
     * puts arm hinge to outake position
     */
    public void armHingeOut() {
        armToPosition(ARM_HINGE_OUT_POSITION);

    }

    /**
     * puts bucket hinge to intake position
     */
    public void bucketHingeIn() {
        bucketToPosition(BUCKET_HINGE_IN_POSITION);

    }

    /**
     * puts bucket hinge to outtake position
     */
    public void bucketHingeOut() {
       bucketToPosition(BUCKET_HINGE_OUT_POSITION);
    }

    /**
     * sets position of the bucket
     * @param bucketPosition - position bucket goes to
     */
    public void bucketToPosition(double bucketPosition) {
        leftBucketHinge.setPosition(bucketPosition);
        rightBucketHinge.setPosition(bucketPosition);
    }

    /**
     * sets position of the arm
     * @param armPosition - position arms go to
     */
    public void armToPosition(double armPosition) {
        leftArmHinge.setPosition(armPosition);
        rightArmHinge.setPosition(armPosition);
    }

    /**
     *
     * Systems Check
     */
    public void systemsCheck(AIMPad aimpad) {
        loop(aimpad);
        if (aimpad.isAPressed()) {
            setActiveArmState(ArmState.ARMIN);
        } else if (aimpad.isBPressed()) {
            setActiveArmState(ArmState.ARMOUT);
        } else if (aimpad.isLeftStickMovementEngaged()) {
            setArmStateCustom(aimpad.getLeftStickX());
        }

        if (aimpad.isXPressed()) {
            setActiveBucketState(BucketState.BUCKETIN);
        } else if (aimpad.isYPressed()) {
            setActiveBucketState(BucketState.BUCKETOUT);
        } else if (aimpad.isRightStickMovementEngaged()) {
            setBucketStateCustom(aimpad.getRightStickX());
        }
    }

}

