package org.firstinspires.ftc.teamcode.Auto;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SelectMenuAuto {
    private final GamepadEx gamepad;
    private final Telemetry telemetry;

    private int currentSelection = 1;

    private boolean allianceSelected = false;
    private ALLIANCES selectedAlliance = null;

    private boolean sideSelected = false;
    private SIDES selectedSide = null;

    private boolean parkingSelected = false;
    private PARKING_SPOTS selectedParking = null;

    private boolean stopRequested;

    private ButtonReader UpReader;
    private ButtonReader DownReader;
    private ButtonReader AReader;

    public enum ALLIANCES {
        RED,
        BLUE
    }

    public enum SIDES {
        LEFT,
        RIGHT
    }

    public enum PARKING_SPOTS {
        ASCEND,
        OBSERVATION
    }

    public SelectMenuAuto(GamepadEx pad, Telemetry telemetry1, boolean stopRequested){
        gamepad = pad;
        telemetry = telemetry1;
        this.stopRequested = stopRequested;

        UpReader = new ButtonReader(gamepad, GamepadKeys.Button.DPAD_UP);
        DownReader = new ButtonReader(gamepad, GamepadKeys.Button.DPAD_DOWN);
        AReader = new ButtonReader(gamepad, GamepadKeys.Button.A);
    }

    private ALLIANCES getCurAlliance(int num){
        switch(num){
            case 1:
                return ALLIANCES.RED;
            case 2:
                return ALLIANCES.BLUE;
        }
        return null;
    }

    private SIDES getCurSide(int num){
        switch(num){
            case 1:
                return SIDES.LEFT;
            case 2:
                return SIDES.RIGHT;
        }
        return null;
    }

    private PARKING_SPOTS getCurParking(int num){
        switch(num){
            case 1:
                return PARKING_SPOTS.ASCEND;
            case 2:
                return PARKING_SPOTS.OBSERVATION;
        }
        return null;
    }

    public void ShowMenu(boolean stopRequested) {
        this.stopRequested = stopRequested;
        if (!allianceSelected) {
            telemetry.addLine("Press Up/Down on D-pad to select your placement and A confirm!");

            if (UpReader.wasJustReleased() && currentSelection == 1) {
                currentSelection += 1;
            } else if (DownReader.wasJustReleased() && currentSelection == 2) {
                currentSelection -= 1;
            } else if (AReader.wasJustReleased()) {
                allianceSelected = true;
                selectedAlliance = getCurAlliance(currentSelection);
                currentSelection = 1;
            }

            telemetry.addData("Currently Selected Alliance", getCurAlliance(currentSelection));
            telemetry.addData("Currently Selected Side", "None");
            telemetry.addData("Currently Selected Parking Spot", "None");
        } else if (!sideSelected) {
            telemetry.addLine("Press Up/Down on D-pad to select your placement and A confirm!");

            if (UpReader.wasJustReleased() && currentSelection == 1) {
                currentSelection += 1;
            } else if (DownReader.wasJustReleased() && currentSelection == 2) {
                currentSelection -= 1;
            } else if (AReader.wasJustReleased()) {
                sideSelected = true;
                selectedSide = getCurSide(currentSelection);
                currentSelection = 1;
            }

            telemetry.addData("Currently Selected Alliance", selectedAlliance);
            telemetry.addData("Currently Selected Side", getCurSide(currentSelection));
            telemetry.addData("Currently Selected Parking Spot", "None");
        } else if (!parkingSelected) {
            telemetry.addLine("Press Up/Down on D-pad to select your placement and A confirm!");

            if (UpReader.wasJustReleased() && currentSelection == 1) {
                currentSelection += 1;
            } else if (DownReader.wasJustReleased() && currentSelection == 2) {
                currentSelection -= 1;
            } else if (AReader.wasJustReleased()) {
                parkingSelected = true;
                selectedParking = getCurParking(currentSelection);
                currentSelection = 1;
            }

            telemetry.addData("Currently Selected Alliance", selectedAlliance);
            telemetry.addData("Currently Selected Side", selectedSide);
            telemetry.addData("Currently Selected Parking Spot", getCurParking(currentSelection));
        } else {
            telemetry.addData("Currently Selected Alliance", selectedAlliance);
            telemetry.addData("Currently Selected Side", selectedSide);
            telemetry.addData("Currently Selected Parking Spot", selectedParking);
        }
    }

    public SIDES getSide(){
        if (sideSelected) {
            return selectedSide;
        } else{
            return null;
        }
    }

    public ALLIANCES getAlliance(){
        if (allianceSelected) {
            return selectedAlliance;
        } else {
            return null;
        }
    }

    public PARKING_SPOTS getParkingSpot(){
        if (parkingSelected) {
            return selectedParking;
        } else {
            return null;
        }
    }
}
// auto fix