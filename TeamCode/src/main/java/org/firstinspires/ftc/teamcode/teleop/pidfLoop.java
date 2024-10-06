package org.firstinspires.ftc.teamcode.teleop;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;

public class pidfLoop {
    public static double p,i,d;
    private PIDController controller = new PIDController(p,i,d);
    public static double f;

    private final double ticks_in_degree = 100/180.0;

    public Action getLoop(DcMotor arm, double target) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                controller.setPID(p,i,d);

                int armPos = arm.getCurrentPosition();
                double pid = controller.calculate(armPos, target);
                double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

                double power = pid + ff;

                arm.setPower(power);
                return false;
            }
        };
    }
}
