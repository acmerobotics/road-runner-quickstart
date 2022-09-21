package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import org.firstinspires.ftc.teamcode.tfrec.Detector;
import org.firstinspires.ftc.teamcode.tfrec.classification.Classifier;

@TeleOp(name = "Vision Test")

public class visionTest extends LinearOpMode {


    private Detector tfDetector = null;
    private ElapsedTime runtime = new ElapsedTime();
    private static String MODEL_FILE_NAME = "model_unquant.tflite";
    private static String LABEL_FILE_NAME = "labels.txt";
    private static Classifier.Model MODEL_TYPE = Classifier.Model.FLOAT_EFFICIENTNET;


    @Override
    public void runOpMode() {
        try {
            try {
                tfDetector = new Detector(MODEL_TYPE, MODEL_FILE_NAME, LABEL_FILE_NAME, hardwareMap.appContext, telemetry);
                tfDetector.activate();
            } catch (Exception ex) {
                telemetry.addData("Error", String.format("Unable to initialize Detector. %s", ex.getMessage()));
                sleep(3000);
                return;
            }
            telemetry.addData("Detector", "Ready");
            telemetry.update();
            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();

            while (opModeIsActive()) {
                List<Classifier.Recognition> results = tfDetector.getLastResults();
                if (results == null || results.size() == 0) {
                    telemetry.addData("Info", "No results");
                } else {
                    for (Classifier.Recognition r : results) {
                        String item = String.format("%s: %.2f", r.getTitle(), r.getConfidence());
                        telemetry.addData("Found", item);

                    }
                }
                telemetry.update();
            }
        } catch (Exception ex) {
            telemetry.addData("Init Error", ex.getMessage());
            telemetry.update();
        } finally {
            if (tfDetector != null) {
                tfDetector.stopProcessing();
            }

        }

    }
}