package org.firstinspires.ftc.teamcode;
// ToDo: modify MecanumDrive to include an isBusy() method.

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="ConceptTractorBeam", group="testing")
public class ConceptTractorBeam extends LinearOpMode {
    AutoFluffy fluffy;
    MecanumDrive drive;

    final Vector2d YELLOW_DELIVERY_FINAL_POINT = new Vector2d(0,0);
    public class GoToYellowDelivery implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            //drive.pose = fluffy.getPoseFromAprilTag();
            drive.localizer.update();
            //return drive.localizer.isBusy();
            return false;
        }
    }

    public Action goToYellowDelivery() {
        return new GoToYellowDelivery();
    }

    public void runOpMode() {
        fluffy = new AutoFluffy(this);
        drive = fluffy.drive;
        // ... get to YELLOW_DELIVERY_INITIAL_POINT
        Actions.runBlocking(
                new ParallelAction(
                        //drive.actionBuilder(drive.pose).
                        //        strafeTo(YELLOW_DELIVERY_FINAL_POINT),
                        goToYellowDelivery())
        );
    }
}



