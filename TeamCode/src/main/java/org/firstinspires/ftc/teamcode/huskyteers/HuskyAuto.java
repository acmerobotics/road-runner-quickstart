package org.firstinspires.ftc.teamcode.huskyteers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.huskyteers.vision.HuskyVision;

import java.util.List;
import java.util.Optional;

@Config
@Autonomous(name = "Husky Auto", group = "Auto")
public class HuskyAuto extends HuskyBot {
    public static int MAX_TRIES = 20;

    /**
     * Let's arbitrarily assign numbers to each side it could be based on the location relative
     * to the robot's starting position.
     * <pre>
     *      1
     *    ______
     * 0 |     |  2
     *   |     |
     *    robot start
     *  </pre>
     *
     * @return Location of team prop, or -1 if not found
     */
    public int detectTeamPropLocation() {
        // TODO: Change this to maybe be in seconds, instead of number of tries which could be different every time.
        int tries = 0;
        do {
            List<Recognition> recognitions = huskyVision.tensorflowdetection.tfod.getRecognitions();
            Optional<Recognition> likelyRecognition = recognitions.stream().filter(recognition -> recognition.getLabel().equals("HuskyProp")).findAny();
            if (likelyRecognition.isPresent()) {
                Recognition recognition = likelyRecognition.get();
                // TODO: Better recognition algorithm, currently classifying based on the 1/3 of the screen that it is found in.
                return (int) (HuskyVision.WIDTH / (recognition.getLeft() + recognition.getRight()) / 2);
            }
            // TODO: Add some code maybe to move the robot a little to improve chances of finding team prop instead of trying same thing 20 times
        } while (++tries <= MAX_TRIES);
        return -1;
    }

    public void navigateToTeamPropLocation(int location) {
        // TODO: Implement navigating to the team prop location
        throw new UnsupportedOperationException();
    }

    public int locationToAprilTag(int location) {
        if (location == 0) {
            return 432;
            // TODO: Replace with actual values, depending on alliance too.
        }
        return -1;
    }

    public void parkInBackstage() {
        // TODO: Implement navigating to backstage
        throw new UnsupportedOperationException();
    }

    @Override
    public void runOpMode() {
        super.initializeHardware();

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            int teamPropLocation = detectTeamPropLocation();
            if (teamPropLocation != -1) {
                // Put down purple pixel
                navigateToTeamPropLocation(teamPropLocation);
                moveClawToBottom();
                openClaw();
                // TODO: Pick up yellow pixel
                // Put yellow pixel on backdrop
                alignWithAprilTag(locationToAprilTag(teamPropLocation));
                moveClawToBackdropPosition();
                openClaw();
                parkInBackstage();
            }
        }
    }
}
