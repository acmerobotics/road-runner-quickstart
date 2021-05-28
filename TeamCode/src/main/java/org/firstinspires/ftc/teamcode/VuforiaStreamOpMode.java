package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/*
 * This sample demonstrates how to stream frames from Vuforia to the dashboard. Make sure to fill in
 * your Vuforia key below and select the 'Camera' preset on top right of the dashboard. This sample
 * also works for UVCs with slight adjustments.
 */
@Autonomous
public class VuforiaStreamOpMode extends LinearOpMode {

    // TODO: fill in
    public static final String VUFORIA_LICENSE_KEY =
            "AYyrYgn/////AAABmfoVAabL9068k4+3G7BnkUsn3CiCAh0CExHOl7JMywbgqm+E6EGnvUJu4GmpbxVoJT9fzwRQVJtOg0/0UM8utTn6W+zUBcMxW/pVpPa5yaL3dhcgq8z58hV0f1dbDYJxcBlikT49wSdyoZlG/Cs3HSpplIuK/3xAqBUUYjqFfQ754AYtY2eCz2HWjnAB9IHcHE3MX9TRmEsJMGI6ZMkbMp7N5kxlZDX3yL+36sekpn3NdYfuvDPrQD7DEbGYEYHg8FERjE6vpx1OhU6sLAfUVN3o40Qrct/G3pKndmMJ1MnvAVXPE2llAmXvdQwTNMYRq/BXj446ICd3eCCXj9kozasAra6fJRkn9jFk1FOE4OVS";

    @Override
    public void runOpMode() throws InterruptedException {
        // gives Vuforia more time to exit before the watchdog notices
        msStuckDetectStop = 2500;

        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuforiaParams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);

        FtcDashboard.getInstance().startCameraStream(vuforia, 0);

        waitForStart();

        while (opModeIsActive());
    }
}
