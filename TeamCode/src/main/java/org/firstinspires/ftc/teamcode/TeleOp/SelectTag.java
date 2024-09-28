package org.firstinspires.ftc.teamcode.TeleOp;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SelectTag {
    private final Gamepad gamepad;
    private final Telemetry telemetry;
    private int selectedTag = 1;
    private boolean stopRequested;

    public SelectTag(Gamepad pad, Telemetry telemetry1, boolean stoprequested){
        gamepad = pad;
        telemetry = telemetry1;
        stopRequested = stoprequested;
    }



    public void ShowMenu(boolean stopRequested) throws InterruptedException {
        this.stopRequested = stopRequested;

        telemetry.addLine("Press Up/Down on D-pad to select your tag!");
        if (gamepad.dpad_up && selectedTag != 6) {
            selectedTag += 1;
        } else if (gamepad.dpad_down && selectedTag != 1) {
            selectedTag -= 1;
        }
        telemetry.addData("Currently Selected Tag", selectedTag);
    }

    public int getTag(){
        return selectedTag;
    }

}