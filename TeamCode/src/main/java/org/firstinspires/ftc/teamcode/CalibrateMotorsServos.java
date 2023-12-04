package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.List;

@TeleOp(name="CalibrateMotorsServos", group="testing")
public class CalibrateMotorsServos extends OpMode {
//
    DcMotor liftMotor, hangerMotor;
    final double LIFTPOWER = 0.50;
    Servo dronePusher, grabberRot, finger, hangerLatch;

    enum StateA {READY, FIRST_PRESS, FP_WAIT, MULTI_PRESS, MP_WAIT};
    enum StateB {READY, FIRST_PRESS, FP_WAIT, MULTI_PRESS, MP_WAIT};
    enum StateJoystick {READY, FIRST_PRESS, FP_WAIT, MULTI_PRESS, MP_WAIT};

    final int FP_WAITTIME = 500;
    final int MP_WAITTIME = 100;

    StateA stateA = StateA.READY;
    StateB stateB = StateB.READY;
    StateJoystick stateJoystick = StateJoystick.READY;
    List<CalibratingObject> calibratingObjects;
    int calibrationIndex = 0;

    ElapsedTime timerA=new ElapsedTime(), timerB=new ElapsedTime(), timerJoystick=new ElapsedTime();

    public void init() {
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        liftMotor.setDirection(FORWARD);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(LIFTPOWER);

        hangerMotor = hardwareMap.dcMotor.get("hangerMotor");
        hangerMotor.setDirection(FORWARD);
        hangerMotor.setTargetPosition(0);
        hangerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangerMotor.setPower(LIFTPOWER);

        dronePusher = hardwareMap.servo.get("dronePusher");
        dronePusher.setPosition(0.8);

        grabberRot = hardwareMap.servo.get("grabberRot");
        grabberRot.setPosition(0.0);


        finger = hardwareMap.servo.get("finger");
        //finger.setDirection(Servo.Direction.REVERSE);
        finger.setPosition(0.0);

        hangerLatch = hardwareMap.servo.get("hangerLatch");
        hangerLatch.setDirection(Servo.Direction.FORWARD);
        hangerLatch.setPosition(.7);

        calibratingObjects = Arrays.asList(
            new CalibratingMotor(liftMotor, "liftMotor"),
            new CalibratingMotor(hangerMotor, "hangerMotor", 200.0),
            new CalibratingServo(dronePusher, "dronePusher"),
            new CalibratingServo(grabberRot, "grabberRotation"),
            new CalibratingServo(finger, "finger"),
            new CalibratingServo(hangerLatch, "hangerLatch"));

    }


    public void loop() {
        updateStateA();
        updateStateB();
        updateStateJoystick();
        for (CalibratingObject obj : calibratingObjects) {
            obj.setPosition();
        }
        displayTelemetry();
    }

    public void incrementIndex() {
        if (calibrationIndex >= calibratingObjects.size()-1) {
            return;
        }
        calibrationIndex++;
    }

    public void decrementIndex() {
        if (calibrationIndex <= 0) {
            return;
        }
        calibrationIndex--;
    }
    public void updateStateA() {
        switch(stateA) {
            case READY: {
                if (gamepad1.a) {
                    stateA = StateA.FIRST_PRESS;
                }
                break;
            }
            case FIRST_PRESS: {
                timerA.reset();
                incrementIndex();
                stateA = StateA.FP_WAIT;
                break;
            }
            case FP_WAIT: {
                if (!gamepad1.a) {
                    stateA = StateA.READY;
                } else if (timerA.milliseconds() > FP_WAITTIME) {
                    stateA = StateA.MULTI_PRESS;
                }
                break;
            }
            case MULTI_PRESS: {
                timerA.reset();
                incrementIndex();
                stateA = StateA.MP_WAIT;
                break;
            }
            case MP_WAIT: {
                if (!gamepad1.a) {
                    stateA = StateA.READY;
                } else if (timerA.milliseconds() > MP_WAITTIME) {
                    stateA = StateA.MULTI_PRESS;
                }
                break;
            }
        }



    }

    public void updateStateB() {
        switch (stateB) {
            case READY: {
                if (gamepad1.b) {
                    stateB = StateB.FIRST_PRESS;
                }
                break;
            }
            case FIRST_PRESS: {
                timerB.reset();
                decrementIndex();
                stateB = StateB.FP_WAIT;
                break;
            }
            case FP_WAIT: {
                if (!gamepad1.b) {
                    stateB = StateB.READY;
                } else if (timerB.milliseconds() > FP_WAITTIME) {
                    stateB = StateB.MULTI_PRESS;
                }
                break;
            }
            case MULTI_PRESS: {
                timerB.reset();
                decrementIndex();
                stateB = StateB.MP_WAIT;
                break;
            }
            case MP_WAIT: {
                if (!gamepad1.b) {
                    stateB = StateB.READY;
                } else if (timerB.milliseconds() > MP_WAITTIME) {
                    stateB = StateB.MULTI_PRESS;
                }
                break;
            }
        }
    }
    public void updateStateJoystick() {
        double THRESHOLD = 0.1;
        boolean UP = true;
        boolean DOWN = false;

        switch (stateJoystick) {
            case READY: {
                if (Math.abs(gamepad1.right_stick_y) > THRESHOLD) {
                    stateJoystick = StateJoystick.FIRST_PRESS;
                }
                break;
            }
            case FIRST_PRESS: {
                timerJoystick.reset();
                if (gamepad1.right_stick_y < 0) {
                    calibratingObjects.get(calibrationIndex).incrementPosition(UP);
                }
                else {
                    calibratingObjects.get(calibrationIndex).incrementPosition(DOWN);
                }
                stateJoystick = StateJoystick.FP_WAIT;
                break;
            }
            case FP_WAIT: {
                if (Math.abs(gamepad1.right_stick_y) < THRESHOLD) {
                    stateJoystick = StateJoystick.READY;
                } else if (timerJoystick.milliseconds() > FP_WAITTIME) {
                    stateJoystick = StateJoystick.MULTI_PRESS;
                }
                break;
            }
            case MULTI_PRESS: {
                timerJoystick.reset();
                if (gamepad1.right_stick_y < 0) {
                    calibratingObjects.get(calibrationIndex).incrementPosition(UP);
                }
                else {
                    calibratingObjects.get(calibrationIndex).incrementPosition(DOWN);
                }
                stateJoystick = StateJoystick.MP_WAIT;
                break;
            }
            case MP_WAIT: {
                if (!gamepad1.b) {
                    stateJoystick = StateJoystick.READY;
                } else if (timerJoystick.milliseconds() > MP_WAITTIME) {
                    stateJoystick = StateJoystick.MULTI_PRESS;
                }
                break;
            }
        }
    }

    interface CalibratingObject {
        public double report();
        public String str();
        public void setPosition();
        public void incrementPosition(boolean up);   // increments if up; decrements if !up
    }

    public void displayTelemetry() {
        telemetry.clear();
        /*for (int i = 0; i< calibratingObjects.size(); i++) {
            telemetry.addData(calibratingObjects.get(i).str(), calibratingObjects.get(i).report());
        }*/
        for (int i = calibrationIndex - 2; i <= calibrationIndex + 2; i++) {
            String s = "";

            if (i < 0 || i >= calibratingObjects.size()) {
                telemetry.addLine(s);
                continue;
            }
            if (i == calibrationIndex) {
                s += ">";
            } else {
                s += " ";
            }
            s += calibratingObjects.get(i).str() + "\t";
            telemetry.addData(s, calibratingObjects.get(i).report());
            //telemetry.addData("counter", i);

        }
    }
    class CalibratingMotor implements CalibratingObject {
        public final double INCREMENT = 10.0;
        public double position = 0.0;
        public double increment = 0.0;
        DcMotor motor;
        String name;

        public CalibratingMotor(DcMotor m, String n) {
            this.motor = m;
            this.name = n;
            this.increment = INCREMENT;
        }

        public CalibratingMotor(DcMotor m, String n, double inc) {
            this.motor = m;
            this.name = n;
            this.increment = inc;
        }

        public double report() {
            return (double) motor.getCurrentPosition();
        }

        public String str() {
            return name;
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
    }

    class CalibratingServo implements CalibratingObject {
        public final double INCREMENT = 0.05;
        public double position = 0.0;

        Servo servo;
        String name;

        public CalibratingServo(Servo s, String n) {
            this.servo = s;
            this.name = n;
        }

        public double report() {
            return (double)                                                                                                                                                                                                                                                                                                                                      servo.getPosition();
        }

        public String str() {
            return name;
        }

        public void setPosition() {
            servo.setPosition(position);
        }

        public void incrementPosition(boolean up) {
            if (up) {
                position += INCREMENT;
            }
            else {
                position -= INCREMENT;
            }

        }
    }





}
