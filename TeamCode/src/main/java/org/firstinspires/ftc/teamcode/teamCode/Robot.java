package org.firstinspires.ftc.teamcode.teamCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Robot extends Thread {

        public LinearOpMode opMode;
        public ArmController arm;
        public LiftController lift;
        public JointController joint;
        public ClawController claw;

        public void run() {
            while (!isInterrupted()) {
                arm.update();
                lift.update();
            }
        }

        public Robot(LinearOpMode opMode) {
            HardwareMap map = opMode.hardwareMap;
            this.opMode = opMode;
            arm = new ArmController (map);
            lift = new LiftController (map);
            joint = new JointController(map);
            claw = new ClawController (map);

        }
    }
