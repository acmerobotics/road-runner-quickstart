package org.firstinspires.ftc.teamcode.subsystems.claw;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.hardware.ContinuousServo;
import org.firstinspires.ftc.teamcode.util.enums.SampleColors;

import java.util.Arrays;
import java.util.concurrent.TimeUnit;

public class Claw {
    ContinuousServo servo1;
    ContinuousServo servo2;
    RevColorSensorV3 colorSensor;

    ElapsedTime sensorTimeout;

    public Claw(ContinuousServo s1, ContinuousServo s2, RevColorSensorV3 sensor) {
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

    public void smartStopIntake(SampleColors... colors) {
        SampleColors s = detectSample();
        if (Arrays.stream(colors).anyMatch(x -> x == s )) {
            stopIntake();
        } else if (s != null) {
            eject();
        }
    }

    /**
     *
     * @param colors colors that you want to stop the intake for
     * @return 0: nothing in intake, 1: intaked targeted color, -1: intaked non-target (eject)
     */

    public int smartStopDetect(SampleColors... colors) {
        SampleColors s = detectSample();
        if (Arrays.stream(colors).anyMatch(x -> x == s )) {
            return 1;
        } else if (s != null) {
            return -1;
        }
        return 0;
    }

    public void eject() {
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
        } else if (colorSensor.getDistance(DistanceUnit.MM) > 3) {
            return SampleColors.YELLOW;
        }

        sensorTimeout.reset();

        return null;
    }
}
