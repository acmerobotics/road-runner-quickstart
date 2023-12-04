package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

public class CalibrateActuators extends OpMode {

    List<CalibratingObject> actuators = new ArrayList<CalibratingObject>();
    int index = 0;
    final int PREVIEW = 2;
    final double THRESHOLD = 0.15;

    public void init() {
        // will this get DcMotorEx?  CRServo?
        for (DcMotor d : hardwareMap.getAll(DcMotor.class)) {
            actuators.add(new CalibratingMotor(d, d.getDeviceName()));
        }

        for (Servo s : hardwareMap.getAll(Servo.class)) {
            actuators.add(new CalibratingServo(s, s.getDeviceName()));
        }
    }

    public void loop() {
        displayDevices();
        updateJoystick();
    }

    public void displayDevices() {
        String s = "";
        telemetry.clear();
        for (int i = index - PREVIEW; i <= index + PREVIEW; i++) {
            if (i >= 0 && i < actuators.size()) {
                if (i == index) {
                    s = ">";
                } else {
                    s = " ";
                }
                s += actuators.toString();
            } else s = " ";

            telemetry.addLine(s);
        }
        telemetry.update();
    }

    public void updateJoystick() {
        // TODO: state machine to change cursor
        if (gamepad1.left_stick_y > THRESHOLD) {
            index = (index <= 0)? 0: index-1;
        }
        else if (gamepad1.left_stick_y < -THRESHOLD) {
            index = (index >= actuators.size()-1)? actuators.size()-1:index+1;
        }
    }
    interface CalibratingObject {
        public void setPosition();
        public void incrementPosition(boolean up);   // increments if up; decrements if !up
        public void setIncrement(double increment);

        public double getIncrement();
    }

    class CalibratingMotor implements CalibrateActuators.CalibratingObject {


        public double position = 0.0;
        double increment = 10;
        DcMotor motor;
        String name;

        public CalibratingMotor(DcMotor m, String n) {
            this.motor = m;
            this.name = n;
        }


        @Override
        public String toString() {
            String s = name + "\t" + motor.getCurrentPosition();

            if (this.motor.getDirection() == DcMotorSimple.Direction.REVERSE) {
                s+= " (Rev)";
            }
            return s;
        }
        public void setPosition() {
            motor.setTargetPosition((int) position);
        }

        public void incrementPosition(boolean up) {
            if (up) {
                position += increment;
            }
            else {
                position -= increment;
            }

        }

        public void setIncrement(double increment) {
            if (increment <= 0) {return;}
            else {this.increment = increment;}
        }

        public double getIncrement() {
            return this.increment;
        }

        public void setForward() {
            this.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public void setReverse() {
            this.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public void toggleDirection() {
            if (this.motor.getDirection() == DcMotorSimple.Direction.FORWARD) {
                setReverse();
            }
            else {setForward();}
        }




    }

    class CalibratingServo implements CalibrateActuators.CalibratingObject {

        double increment = 0.05;
        public double position = 0.0;

        Servo servo;
        String name;

        public CalibratingServo(Servo s, String n) {
            this.servo = s;
            this.name = n;
        }

        @Override
        public String toString() {
            String s = name + "\t" + servo.getPosition();
            if (servo.getDirection() == REVERSE) {
                s += " (Rev)";
            }
            return s;
        }

        public void setPosition() {
            servo.setPosition(position);
        }

        public void incrementPosition(boolean up) {
            if (up) {
                position += increment;
            }
            else {
                position -= increment;
            }

        }

        public void setIncrement(double increment) {
            if (increment <=0 ) {return;}
            else {this.increment = increment;}
        }

        public double getIncrement() {
            return this.increment;
        }

        public void setForward() {
            servo.setDirection(FORWARD);
        }

        public void setReverse() {
            servo.setDirection(Servo.Direction.REVERSE);
        }

        public void toggleDirection() {
            if (servo.getDirection() == FORWARD) {
                setReverse();
            }
            else {setForward();}
        }


    }

}
