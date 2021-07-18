package org.firstinspires.ftc.teamcode.drive.SampleAutonPaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SampleAutonPathNear extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            " AYatEuv/////AAABmT+AWmCDTkv2vYyblEvtrsNLJAlf7wR2LkHEsYDRCtrwvXhJLDMukOX18IhahgnbE2S2Nlw1HC1qHid4Yjhco1+ynBT2FzfJnITxCwFSWlmZvRrXch2E++2mJtvRVcjCJrbjq4wcbcxzRykkPRCTjgGjfWa4W/JmbRstY8+nUZ5f7La0854LYFwtEHJjnjyUyD+caXuipBG06UInhY0HYoQvwQlg4SIG42AHJHQ6MQa7iuCu10+ycOf3VuBdh2QdjzZkcylXsPtx49pLN8+LKFlBHuo40g3dzaNmzPE9Iogd50C/SN5LezkEGd9EvVBJPbrrUZyXGuAtb0WPY1Cp635tk3SjfgzspU5/dZ4TXuOs ";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive2 = new HardwareFile(hardwareMap);
        Pose2d startPose = new Pose2d(-63, -48, Math.toRadians(10));

        drive.setPoseEstimate(startPose);
        /*Trajectory trajoutpow = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-25, -24), Math.toRadians(5))
                .build();

        Trajectory trajmidpow = drive.trajectoryBuilder(trajoutpow.end())
                .splineTo(new Vector2d(-25, -14), Math.toRadians(5))
                .build();

        Trajectory trajinpow = drive.trajectoryBuilder(trajmidpow.end())
                .splineTo(new Vector2d(-25, -4), Math.toRadians(5))
                .build();
*/
        Trajectory trajpickup = drive.trajectoryBuilder(startPose)
                /*.splineTo(new Vector2d(-20 , -40),  Math.toRadians(0))
                .splineTo(new Vector2d(-50,-48), 0)*/
                .splineTo(new Vector2d(-12,-30), Math.toRadians(5))
                .build();

        Trajectory traj0ring = drive.trajectoryBuilder(trajpickup.end())
                .splineTo(new Vector2d(12 , -40),  Math.toRadians(90))
                .build();

        Trajectory traj0line = drive.trajectoryBuilder(traj0ring.end())
                .splineTo(new Vector2d(0 , -24),  Math.toRadians(90))
                .build();

        Trajectory traj1ring = drive.trajectoryBuilder(trajpickup.end())
                .splineTo(new Vector2d(18 , -30),  Math.toRadians(180))
                .build();

        Trajectory traj1line = drive.trajectoryBuilder(traj1ring.end())
                .splineTo(new Vector2d(0 , -24),  Math.toRadians(90))
                .build();

        Trajectory traj4ring = drive.trajectoryBuilder(trajpickup.end())
                .splineTo(new Vector2d(48 , -36),  Math.toRadians(135))
                .build();

        Trajectory traj4line = drive.trajectoryBuilder(traj4ring.end())
                .splineTo(new Vector2d(0 , -24),  Math.toRadians(90))
                .build();

        initVuforia();
        initTfod();
        camera();
        drive2.grab();
        waitForStart();
        if (isStopRequested()) return;
        camera();
        drive2.leftIntakeHolder.setPosition(1);
        drive2.rightIntakeHolder.setPosition(0);
        sleep(2500);
        drive2.leftIntakeHolder.setPosition(0);
        drive2.rightIntakeHolder.setPosition(1);
        /*drive2.shooterflap.setPosition(0.4) ;*/
        drive.followTrajectory(trajpickup);
        drive2.leftIntakeHolder.setPosition(0.8);
        drive2.rightIntakeHolder.setPosition(0.8);
        shooter(3);
        //
        // sleep(5000);
        //drive.followTrajectory(trajmidpow);
        //shooter(1);
        //drive.followTrajectory(trajinpow);
       // shooter(1);
        //0 ring
        drive2.intake(1);
        if(height==0) {
            drive2.magdown();
            drive.followTrajectory(traj0ring);
            wobbledrop();
            drive.followTrajectory(traj0line);
            drive2.leftIntakeHolder.setPosition(1);
            drive2.rightIntakeHolder.setPosition(0);
        }else if(height==1){
            drive2.magdown();
            /*drive.followTrajectory(trajpickup);
            shooter(3);*/
            drive2.magdown();
            drive.followTrajectory(traj1ring);
            wobbledrop();
            drive.followTrajectory(traj1line);
            drive2.leftIntakeHolder.setPosition(1);
            drive2.rightIntakeHolder.setPosition(0);
        }else if(height==4){
            drive2.magdown();
            drive.followTrajectory(traj4ring);
            wobbledrop();
            drive.followTrajectory(traj4line);
            drive2.leftIntakeHolder.setPosition(1);
            drive2.rightIntakeHolder.setPosition(0);
        }
    }
    public static  HardwareFile drive2;
    public static int height;
    public void camera(){
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0/9.0);
        }
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    //telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                    // recognition.getLeft(), recognition.getTop());
                    //telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                    // recognition.getRight(), recognition.getBottom());
                    telemetry.addData("",recognition.getLabel()=="Quad");
                    if (recognition.getLabel().equals("Quad")){
                        height=4;
                    }else if(recognition.getLabel().equals("Single")){
                        height=1;
                    }else{
                        height=0;
                    }
                }
                //height=updatedRecognitions.size();
                telemetry.addData("Height: ", height);
                telemetry.update();
            }
        }

    }
    public void wobbledrop(){
        drive2.grab();
        sleep(100);
        drive2.wobbleArmDown();
        sleep(1000);
        drive2.release();
        sleep(100);
    }
    public void shooter(int rounds){
        drive2.shooter(1);
        sleep(2000);
        for(int i=0;i<=rounds;++i){
            drive2.magup();
            drive2.magup();
            drive2.slapper.setPosition(0.35);
            sleep(100);
            drive2.slapper.setPosition(0.5);
            sleep(1000);
        }
        drive2.shooter(0);
        drive2.magdown();
    }
}
