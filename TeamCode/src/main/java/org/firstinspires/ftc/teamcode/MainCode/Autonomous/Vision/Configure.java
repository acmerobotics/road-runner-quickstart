package org.firstinspires.ftc.teamcode.MainCode.Autonomous.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name="Configure OpenCV", group="Linear Opmode")
public class Configure extends LinearOpMode {
    enum Parameter {
        HUE,
        SAT,
        VAL,
        POS
    }


    Parameter parameterToModify = Parameter.HUE;

    boolean canSwitch = true;

    public static double MODIFY_SPEED = 8;
    public static double MOVE_SPEED = 8;

    public void runOpMode(){
//        Class<? extends  RegionParameter> a = {LeftRegion.class};

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        // I have no clue what this does
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        FtcDashboard.getInstance().startCameraStream(camera, 30);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                camera.startStreaming(VisionParameters.resX,VisionParameters.resY, OpenCvCameraRotation.UPRIGHT);
            }
            public void onError(int errorCode) {}
        });

        ElementDetectionPipeline elementDetectionPipeline = new ElementDetectionPipeline();
        camera.setPipeline(elementDetectionPipeline);

        waitForStart();

        while(opModeIsActive()){
            // Loading config with x, b, y, a
            {
                if (gamepad1.x) {
                    elementDetectionPipeline.setColorParameters(
                            VisionParameters.blueHueMin,
                            VisionParameters.blueHueMax,
                            VisionParameters.blueSatMin,
                            VisionParameters.blueSatMax,
                            VisionParameters.blueValMin,
                            VisionParameters.blueValMax
                    );
                }
                if (gamepad1.b) {
                    elementDetectionPipeline.setColorParameters(
                            VisionParameters.redHueMin,
                            VisionParameters.redHueMax,
                            VisionParameters.redSatMin,
                            VisionParameters.redSatMax,
                            VisionParameters.redValMin,
                            VisionParameters.redValMax
                    );
                }
                if (gamepad1.y) {
                    elementDetectionPipeline.setPositionParameters(
                            VisionParameters.middleStartX,
                            VisionParameters.middleStartY,
                            VisionParameters.middleEndX,
                            VisionParameters.middleEndY
                    );
                }
                if (gamepad1.a) {
                    elementDetectionPipeline.setPositionParameters(
                            VisionParameters.leftStartX,
                            VisionParameters.leftStartY,
                            VisionParameters.leftEndX,
                            VisionParameters.leftEndY
                    );
                }
            }

            // Change parameter to edit with dpad
            {
                if(canSwitch){
                    if(gamepad1.dpad_right){
                        switch(parameterToModify){
                            case HUE:
                                parameterToModify = Parameter.SAT;
                                break;
                            case SAT:
                                parameterToModify = Parameter.VAL;
                                break;
                            case VAL:
                                parameterToModify = Parameter.POS;
                                break;
                            case POS:
                                parameterToModify = Parameter.HUE;
                                break;
                        }
                    }
                    if(gamepad1.dpad_left){
                        switch(parameterToModify){
                            case HUE:
                                parameterToModify = Parameter.POS;
                                break;
                            case SAT:
                                parameterToModify = Parameter.HUE;
                                break;
                            case VAL:
                                parameterToModify = Parameter.SAT;
                                break;
                            case POS:
                                parameterToModify = Parameter.VAL;
                                break;
                        }
                    }
                }
                canSwitch = !gamepad1.dpad_left && !gamepad1.dpad_right;
            }

            // Modify parameters with sticks
            {
                switch(parameterToModify){
                    case HUE:
                        elementDetectionPipeline.minHue -= MODIFY_SPEED*gamepad1.left_stick_y;
                        elementDetectionPipeline.maxHue -= MODIFY_SPEED*gamepad1.right_stick_y;
                        break;
                    case SAT:
                        elementDetectionPipeline.minSat -= MODIFY_SPEED*gamepad1.left_stick_y;
                        elementDetectionPipeline.maxSat -= MODIFY_SPEED*gamepad1.right_stick_y;
                        break;
                    case VAL:
                        elementDetectionPipeline.minVal -= MODIFY_SPEED*gamepad1.left_stick_y;
                        elementDetectionPipeline.maxVal -= MODIFY_SPEED*gamepad1.right_stick_y;
                        break;
                    case POS:
                        elementDetectionPipeline.xStart += MOVE_SPEED*gamepad1.left_stick_x;
                        elementDetectionPipeline.yStart += MOVE_SPEED*gamepad1.left_stick_y;
                        elementDetectionPipeline.xEnd += MOVE_SPEED*gamepad1.right_stick_x;
                        elementDetectionPipeline.yEnd += MOVE_SPEED*gamepad1.right_stick_y;
                        break;
                }
            }

            // Logging parameters to telemetry
            {
                dashboardTelemetry.addData("Amount", elementDetectionPipeline.amount);
                dashboardTelemetry.addData("Modifying parameter", parameterToModify);
                dashboardTelemetry.addData("hueMin", elementDetectionPipeline.minHue);
                dashboardTelemetry.addData("hueMax", elementDetectionPipeline.maxHue);
                dashboardTelemetry.addData("satMin", elementDetectionPipeline.minSat);
                dashboardTelemetry.addData("satMax", elementDetectionPipeline.maxSat);
                dashboardTelemetry.addData("valMin", elementDetectionPipeline.minVal);
                dashboardTelemetry.addData("valMax", elementDetectionPipeline.maxVal);
                dashboardTelemetry.addData("startX", elementDetectionPipeline.xStart);
                dashboardTelemetry.addData("startY", elementDetectionPipeline.yStart);
                dashboardTelemetry.addData("endX", elementDetectionPipeline.xEnd);
                dashboardTelemetry.addData("endY", elementDetectionPipeline.yEnd);
                dashboardTelemetry.update();
            }

            sleep(100);
        }


    }
}
