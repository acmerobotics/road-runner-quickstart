package org.firstinspires.ftc.teamcode.Auto.Vision;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Teleop.Teleop;

import java.nio.file.Paths;
import java.util.List;


public class Limelight {
    public Limelight3A limelight;
    public static double StrafeAmount = 0;

    public Limelight(HardwareMap HWMap) {
        limelight = HWMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();

    }

//need limeight to move until degree = 0
    public class Align implements Action {
        private double x;
        private double y;

        public Align(double x, double y) {
            this.x = x;
            this.y = y;

        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            return true;
        }
    }

    public Action align(double x, double y) {
        return new Align(x, y);

    }

    public double update(double x, double y) {
        LLResult result = limelight.getLatestResult();
        double tx;
        double ty;
        if (result != null && result.isValid()) {
            tx = result.getTx() * -1; // How far left or right the target is (degrees)
            //ty = result.getTy() * -1; // How far up or down the target is (degrees)

        }
        else {
            return 0;
        }

        return tx;
    }

    public double StrafeAmount ()

    //move along wall = y pos changes
    //take in position
    //put into method/atcition
    //then strafe

}








