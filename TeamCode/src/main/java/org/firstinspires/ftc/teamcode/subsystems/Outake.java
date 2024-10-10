package org.firstinspires.ftc.teamcode.subsystems;

import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.settings.ConfigurationInfo;

public class Outake extends Mechanism {

    Servo leftArmHinge;
    Servo rightArmHinge;
    Servo leftBucketHinge;
    Servo rightBucketHinge;

    final double ARM_HINGE_IN_POSITION = 0; //arm hinges to intake position
    final double ARM_HiNGE_OUT_POSITION = 0; //arm hinges to outake position
    final double BUCKET_HINGE_IN_POSITION = 0; //bucket hinges to intake position
    final double BUCKET_HINGE_OUT_POSITION = 0; //bucket hinges to outake position



    @Override
    public void init(HardwareMap hwMap) {
        leftArmHinge = hwMap.get(Servo.class, ConfigurationInfo.leftArmHinge.getDeviceName());
        rightArmHinge = hwMap.get(Servo.class, ConfigurationInfo.rightArmHinge.getDeviceName());
        leftBucketHinge = hwMap.get(Servo.class, ConfigurationInfo.leftBucketHinge.getDeviceName());
        rightBucketHinge = hwMap.get(Servo.class, ConfigurationInfo.rightBucketHinge.getDeviceName());

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
        armToPosition(ARM_HiNGE_OUT_POSITION);

    }

    /**
     * puts bucket hinge to intake position
     */
    public void bucketHingeIn() {
        bucketToPosition(BUCKET_HINGE_IN_POSITION);

    }

    /**
     * puts bucket hinge to outake position
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

}

