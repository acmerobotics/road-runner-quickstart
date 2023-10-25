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

    private Servo switchServo = null;

    final double SWITCH_CLOSE_POS = 0.13;

    final double SWITCH_LEASE_ONE_POS = 0.19;

    final double SWITCH_RELEASE_TWO_POS = 0.25;

    final double WRIST_MAX_POS = 1.0; // Maximum rotational position
    final double WRIST_MIN_POS = 0.0;  // Minimum rotational position
    final double WRIST_POS_INTAKE = 0.38;
    final double WRIST_POS_INIT = 0.1;

    // arm servo variables, not used in current prototype version.
    public DcMotor armMotor = null;
    final int ARM_POS_INTAKE = 0;
    final int ARM_POS_INT = 3350;

    /**
     * Init slider motors hardware, and set their behaviors.
     * @param hardwareMap the Hardware Mappings.
     * @param armMotorName the name string for arm servo motor
     * @param wristMotorName the name string for wrist servo motor
     */
    public intakeUnit(HardwareMap hardwareMap, String armMotorName, String wristMotorName, String fingerMotorName, String switchMotorName) {
        // Save reference to Hardware map
        this.hardwareMap = hardwareMap;

        Logging.log("init motors for finger, wrist and arm.");
        switchServo = hardwareMap.get(Servo.class, switchMotorName);
        switchServo.setPosition(SWITCH_CLOSE_POS);


        fingerServo = hardwareMap.get(Servo.class, fingerMotorName);
        fingerStop();

        wristServo = hardwareMap.get(Servo.class, wristMotorName);
        wristServo.setDirection(Servo.Direction.FORWARD);
        wristServo.setPosition(WRIST_POS_INTAKE);
        sleep(200);

        armMotor = hardwareMap.get(DcMotor.class, armMotorName);
        resetArmEncoder();
    }

    /**
     * set the target position of wrist servo motor
     * @param wristPos the target position value for wrist servo motor
     */
    private void setWristPosition(double wristPos) {
        wristPos = Range.clip(wristPos, WRIST_MIN_POS, WRIST_MAX_POS);
        wristServo.setPosition(wristPos);
    }

    /**
     * set the target position of arm servo motor
     * @param armPos the target position value for arm servo motor
     */
    public void setArmCountPosition(int armPos) {
        int ARM_MIN_COUNT_POS = -5000;
        int ARM_MAX_COUNT_POS = 5000;
        armPos = Range.clip(armPos, ARM_MIN_COUNT_POS, ARM_MAX_COUNT_POS);
        armMotor.setTargetPosition(armPos);
    }

    public void intakePositions() {
        setArmCountPosition(ARM_POS_INTAKE);
        wristServo.setPosition(WRIST_POS_INTAKE);
        switchServoClose();
    }

    /**
     * set the wrist servo motor position to open the wrist
     */
    public void wristUp() {
        setWristPosition(wristServo.getPosition() + 0.001);
    }

    /**
     * set the wrist servo motor position to close the wrist
     */
    public void wristDown() {
        setWristPosition(wristServo.getPosition() - 0.001);
    }

    public void switchServoOpen() {
        switchServo.setPosition(switchServo.getPosition() + 0.0005);
    }
    public void switchServoClose() {
        switchServo.setPosition(SWITCH_CLOSE_POS);
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

    public double getFingerPosition() {
        return fingerServo.getPosition();
    }

    public double getSwitchPosition() {
        return switchServo.getPosition();
    }

    public void resetArmEncoder() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setArmCountPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.95);
    }

    public void armLift() {
        setArmCountPosition(armMotor.getCurrentPosition() + 10);
    }

    public void armDown() {
        setArmCountPosition(armMotor.getCurrentPosition() - 10);
    }

    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}