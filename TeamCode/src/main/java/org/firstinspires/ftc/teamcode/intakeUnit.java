/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;


/**
 * This is NOT an opmode.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are case sensitive.
 * Motors type: motors for arm and wrist, finger.
 *
 * 1. Arm servo motor: ArmServo
 * 2. wrist servo motor: wristServo
 */
public class intakeUnit
{
    //private
    HardwareMap hardwareMap =  null;

    // wrist servo motor variables
    private Servo wristServo = null;
    private Servo fingerServo = null;
    final double CLAW_OPEN_POS = 0.0;
    final double CLAW_CLOSE_POS = 0.47;
    final double CLAW_MAX_POS = 1; // Maximum rotational position
    final double CLAW_MIN_POS = 0;  // Minimum rotational position

    // arm servo variables, not used in current prototype version.
    public DcMotor armMotor = null;
    final double ARM_SWING_FORWARD = 0.395;
    final double ARM_SWING_LEFT = 0.73;
    final double ARM_SWING_RIGHT = 0.06;

    final double ARM_LOCATION_ADJ = 0.135;
    final double ARM_FLIP_FRONT_LOAD_POS = ARM_LOCATION_ADJ + 0.25;
    final double ARM_FLIP_FRONT_UNLOAD_POS = ARM_LOCATION_ADJ + 0.35;
    final double ARM_FLIP_CENTER = ARM_LOCATION_ADJ + 0.62;
    final double ARM_FLIP_BACK_UNLOAD_PRE = ARM_LOCATION_ADJ + 0.83;
    final double ARM_FLIP_BACK_UNLOAD_POS = ARM_LOCATION_ADJ + 0.87;
    final double ARM_FLIP_BACK_LOAD_POS = 1.0;

    /**
     * Init slider motors hardware, and set their behaviors.
     * @param hardwareMap the Hardware Mappings.
     * @param armMotorName the name string for arm servo motor
     * @param wristMotorName the name string for wrist servo motor
     */
    public intakeUnit(HardwareMap hardwareMap, String armMotorName, String wristMotorName, String fingerMotorName) {
        // Save reference to Hardware map
        this.hardwareMap = hardwareMap;

        Logging.log("init motors for arm and wrist.");
        armMotor = hardwareMap.get(DcMotor.class, armMotorName);

        wristServo = hardwareMap.get(Servo.class, wristMotorName);

        fingerServo = hardwareMap.get(Servo.class, fingerMotorName);
    }

    /**
     * set the target position of wrist servo motor
     * @param wristPos the target position value for wrist servo motor
     */
    private void setWristPosition(double wristPos) {
        wristPos = Range.clip(wristPos, CLAW_MIN_POS, CLAW_MAX_POS);
        wristServo.setPosition(wristPos);
    }

    /**
     * set the target position of arm servo motor
     * @param armPos the target position value for arm servo motor
     */
    public void setArmCountPosition(int armPos) {
        int ARM_MIN_COUNT_POS = -2000;
        int ARM_MAX_COUNT_POS = 2000;
        armPos = Range.clip(armPos, ARM_MIN_COUNT_POS, ARM_MAX_COUNT_POS);
        armMotor.setTargetPosition(armPos);
    }

    public void setArmPosition(double armPos) {
    }

    /**
     * set the wrist servo motor position to open the wrist
     */
    public void wristUp() {
        setWristPosition(wristServo.getPosition() + 0.01);
    }

    /**
     * set the wrist servo motor position to close the wrist
     */
    public void wristDown() {
        setWristPosition(wristServo.getPosition() - 0.01);
    }

    public void fingerIntake() {
        fingerServo.setPosition(0);
    }
    public void fingerStop() {
        fingerServo.setPosition(0.5);
    }
    public void fingerOuttake() {
        fingerServo.setPosition(1.0);
    }
    /**
     * Get the arm servo motor current position value
     * @return the current arm servo motor position value
     */
    public double getArmPosition() {
        return armMotor.getCurrentPosition();
    }

    /**
     * Get the wrist servo motor current position value
     * @return the current wrist servo motor position value
     */
    public double getWristPosition() {
        return wristServo.getPosition();
    }

    /**
     * Manual control arm position
     * @param updatePosition the value needed to add to current arm servo position value.
     */
    public void armManualMoving(int updatePosition) {
        setArmCountPosition(armMotor.getCurrentPosition() + updatePosition);
    }

    public void resetArmEncoder() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setArmCountPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor.setPower(0.95);
    }

    public void armLift() {
        setArmCountPosition(armMotor.getCurrentPosition() + 40);
    }

    public void armDown() {
        setArmCountPosition(armMotor.getCurrentPosition() - 40);
    }
}