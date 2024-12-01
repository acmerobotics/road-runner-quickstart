package org.firstinspires.ftc.teamcode.helpers.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Vector;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class MotorControl {

    private static PIDController liftController, extendoController;
    public final Servo  intakeRight, intakePivot,intakeClaw, intakeTurret, outTakeRight, outTakePivot, outTakeClaw;


    public final Limelight limelight;

    public final ControlledServo intakeLeft;
    public final Lift lift;
    public final Extendo extendo;

    public MotorControl(@NonNull HardwareMap hardwareMap, Telemetry telemetry) {
        lift = new Lift(hardwareMap);
        extendo = new Extendo(hardwareMap);
        intakeLeft = new ControlledServo(hardwareMap, "armleft", "intake", 0.03, 0.115, 0.905);
        intakeRight =  hardwareMap.get(Servo.class, "armright");
        intakeClaw =  hardwareMap.get(Servo.class, "claw");
        intakePivot =  hardwareMap.get(Servo.class, "pivot");
        intakeTurret = hardwareMap.get(Servo.class, "turret");
        outTakeRight = hardwareMap.get(Servo.class, "outtakerotation");
        outTakePivot  = hardwareMap.get(Servo.class, "outtakepivot");
        outTakeClaw = hardwareMap.get(Servo.class, "outtakeclaw");

        limelight = new Limelight(hardwareMap, telemetry);

        intakeLeft.servo.setDirection(Servo.Direction.REVERSE);
    }


    public void update() {
        lift.update();
        extendo.update();
        limelight.telemetry.update();
    }


    public static class Extendo extends ControlledDevice {

        private static double p = 0.0025, i = 0, d = 0.0001;
        /**
         * This initializes the slide motor. This should be run before any other methods.
         *
         * @param hardwareMap The hardware map to use to get the motors.
         */
        public Extendo(HardwareMap hardwareMap) {
            extendoController = new PIDController(p,i,d);
            extendoController.setPID(p,i,d);
            motor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "extendo"));
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setCachingTolerance(0.05);
        }

        /**
         * This stops the slide, sets the state to down, sets the target to 0, and resets the encoder.
         */
        public void reset() {
            motor.setPower(0);
            targetPosition = 0;
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void findZero() {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(-0.3);
            resetting = true;
        }

        /**
         * This updates the slide motor to match the current state. This should be run in a loop.
         */
        public void update() {
            if (resetting){
                if (motor.getCurrent(CurrentUnit.AMPS) > 2.5) {
                    reset();
                    resetting = false;
                }
            }
            else {
                liftController.setPID(p, i, d);
                int pos = motor.getCurrentPosition();
                double pid = extendoController.calculate(pos, targetPosition);

                motor.setPower(pid);
            }
        }



        /**
         * Checks if the motor is close enough to the target position.
         *
         * @return boolean indicating whether the current position is close to the target.
         */
        public boolean closeEnough() {
            // You might want to check both motors if they need to be in sync
            return Math.abs(motor.getCurrentPosition() - targetPosition) < 40 ;
        }
    }

    public static class Lift extends ControlledDevice {
        CachingDcMotorEx motor2;
        CachingDcMotorEx motor3;

        // PID coefficients
        private static double p = 0.005, i = 0, d = 0.00001;

        /**
         * This initializes the lift motors. This should be run before any other methods.
         *
         * @param hardwareMap The hardware map to use to get the motors.
         */
        public Lift(HardwareMap hardwareMap) {
            liftController = new PIDController(p, i, d);
            liftController.setPID(p, i, d);
            motor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "liftr"), 0.005);
            motor2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "liftl"), 0.005);
            motor3 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "liftm"), 0.005);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        /**
         * This stops the lift, sets the state to down, sets the target to 0, and resets the encoder.
         */
        public void reset() {
            motor.setPower(0);
            motor2.setPower(0);
            motor3.setPower(0);
            targetPosition = 0;
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void findZero() {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(-0.3);
            motor2.setPower(-0.3);
            motor3.setPower(-0.3);
            resetting = true;
        }

        /**
         * This updates the lift motors to match the current state. This should be run in a loop.
         */
        public void update() {
            if (resetting) {
                if (motor.getCurrent(CurrentUnit.AMPS) > 2) {
                    reset();
                    resetting = false;
                }
            } else {
                liftController.setPID(p, i, d);
                int pos = motor.getCurrentPosition();

                double power = liftController.calculate(pos, targetPosition);

                motor.setPower(power);
                motor2.setPower(power);
                motor3.setPower(power);
            }
        }

        /**
         * Checks if the motor is close enough to the target position.
         *
         * @return boolean indicating whether the current position is close to the target.
         */
        public boolean closeEnough() {
            return Math.abs(motor.getCurrentPosition() - targetPosition) < 50;
        }
    }


    public abstract static class ControlledDevice {

        public CachingDcMotorEx motor;
        boolean resetting = false;
        double targetPosition;
        public double getTargetPosition() {
            return targetPosition;
        }
        public void setTargetPosition(double targetPosition) {
            this.targetPosition = targetPosition;
        }
        public boolean isResetting() {
            return resetting;
        }


        public abstract void update();
        public abstract void reset();
        public abstract boolean closeEnough();
        public boolean isOverCurrent() {
            return motor.isOverCurrent();
        }
    }


    public static class ControlledServo {
        public CachingServo servo;
        public AnalogInput positionSensor;
        private double targetPosition;
        private final double TOLERANCE, MIN_ANALOG, MAX_ANALOG;

        public ControlledServo(HardwareMap hardwareMap, String servoName, String analogInputName, double tolerance, double min, double max) {
            TOLERANCE = tolerance;
            MIN_ANALOG = min;
            MAX_ANALOG = max;
            servo = new CachingServo(hardwareMap.get(Servo.class, servoName));
            positionSensor = hardwareMap.get(AnalogInput.class, analogInputName);
        }

        public void setTargetPosition(double position) {
            targetPosition = position;
            servo.setPosition(position);
        }

        public double getCurrentPosition() {
            double voltage = positionSensor.getVoltage() / 3.3;
            return Math.abs((voltage - MIN_ANALOG) / (MAX_ANALOG - MIN_ANALOG));
        }


        public boolean closeEnough() {
            return Math.abs(getCurrentPosition() - targetPosition) < TOLERANCE;
        }

        public double getTargetPosition() {
            return targetPosition;
        }
    }


    public static class Limelight {
        private final Limelight3A limelight;
        private final int maxSamples = 20;
        private final List<Double> xSamples;
        private final List<Double> ySamples;
        private boolean isCollectingSamples;
        private final Telemetry telemetry;

        public Limelight(HardwareMap hardwareMap, Telemetry telemetry) {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            xSamples = new ArrayList<>();
            ySamples = new ArrayList<>();
            isCollectingSamples = false;
            this.telemetry = telemetry;

            this.telemetry.addData("limelight", "Started");

            limelight.start(); // Start polling data
        }

        public void stop() {
            limelight.stop();
        }

        public void resetSamples() {
            xSamples.clear();
            ySamples.clear();
            isCollectingSamples = false;
        }

        public boolean isCollectingSamples() {
            return isCollectingSamples;
        }

        public void startCollectingSamples() {
            resetSamples();
            isCollectingSamples = true;
        }

        public boolean collectSamples() {
            telemetry.addData("collecting samples", isCollectingSamples);
            LLResult result = limelight.getLatestResult();
            telemetry.addData("Result", result != null);
            telemetry.addData("Result", result.getPythonOutput() != null);
            if (!isCollectingSamples) {
                return false;
            }

            if (result.getPythonOutput() != null && result.getPythonOutput()[0] != -1) {
                // Get the X_world and Y_world from the Python output
                double xWorld = result.getPythonOutput()[1]; // in cm
                double yWorld = result.getPythonOutput()[2]; // in cm

                // Collect the outputs
                xSamples.add(xWorld);
                ySamples.add(yWorld);

                telemetry.addData("Limelight Detected", true);

                // Keep only the most recent maxSamples
                if (xSamples.size() > maxSamples) {
                    xSamples.remove(0);
                }
                if (ySamples.size() > maxSamples) {
                    ySamples.remove(0);
                }

                // Check if enough samples have been collected
                return xSamples.size() == maxSamples && ySamples.size() == maxSamples;
            }
            telemetry.addData("Limelight Detected", false);
            return false;
        }


        public Vector2d getAveragePoseInInches() {
            // Compute the average X and Y
            double sumX = 0;
            double sumY = 0;
            for (int i = 0; i < maxSamples; i++) {
                sumX += xSamples.get(i);
                sumY += ySamples.get(i);
            }
            double avgX = sumX / maxSamples; // in cm
            double avgY = sumY / maxSamples; // in cm



            // Convert cm to inches
            double avgXInches = avgX * 393.701;
            double avgYInches = avgY * 393.701;

            // Create a Pose2d object with the averaged positions
            return new Vector2d(avgXInches, avgYInches);
        }

        // Additional methods can be added here for more functionalities
    }

}