package org.firstinspires.ftc.teamcode.subsystems.endEffector;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.ContinuousServo;
import org.firstinspires.ftc.teamcode.util.SampleColors;

import java.util.concurrent.TimeUnit;

public class EndEffector {
    ContinuousServo servo1;
    ContinuousServo servo2;
    RevColorSensorV3 colorSensor;

    ElapsedTime sensorTimeout;

    public EndEffector(ContinuousServo s1, ContinuousServo s2, RevColorSensorV3 sensor) {
        servo1 = s1;
        servo2 = s2;
        colorSensor = sensor;

        sensorTimeout = new ElapsedTime();

        colorSensor.initialize();
        colorSensor.enableLed(true);
    }

    public void setPower(float p) {
        servo1.servo.setPower(p);
        servo2.servo.setPower(p);
    }

    public void startIntake() {
        setPower(1);
    }

    public void stopIntake() {
        setPower(0);
    }

    public void smartStopIntake(SampleColors color) {
        if (detectSample() == color) {
            stopIntake();
        }
    }

    public void outtakeSample() {
        setPower((float) -0.5);
    }

    public SampleColors detectSample() {
        if (sensorTimeout.time(TimeUnit.SECONDS) < 1) {
            return null;
        }

        if (colorSensor.blue() > 0.5) {
            return SampleColors.BLUE;
        } else if (colorSensor.red() > 0.5) {
            return SampleColors.RED;
        } else if (colorSensor.getDistance(DistanceUnit.MM) > 5) {
            return SampleColors.YELLOW;
        }

        sensorTimeout.reset();

        return null;
    }
}
