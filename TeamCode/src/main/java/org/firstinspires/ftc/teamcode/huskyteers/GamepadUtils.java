package org.firstinspires.ftc.teamcode.huskyteers;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.function.Function;

public class GamepadUtils {
    public static class Detector {
        Function<Boolean, Void> callback;
        String button;

        Detector(String button, Function<Boolean, Void> callback) {
            this.callback = callback;
            this.button = button;
        }
    }

    private Gamepad currentGamepad1 = new Gamepad();
    private Gamepad previousGamepad1 = new Gamepad();

    private final List<Detector> risingEdgeDetectors;
    private final List<Detector> fallingEdgeDetectors;

    GamepadUtils() {
        this.risingEdgeDetectors = new ArrayList<Detector>();
        this.fallingEdgeDetectors = new ArrayList<Detector>();
    }

    private boolean getButton(String button, Gamepad gamepad) {
        try {
            return (boolean) gamepad.getClass().getField(button).get(gamepad);
        } catch (NoSuchFieldException | IllegalAccessException | NullPointerException e) {
            System.out.println("Invalid button name");
            throw new NoSuchElementException("invalid button");
        }
    }


    /**
     * Watches both rising edge and falling edge.
     *
     * @param detector The Detector with the watched button and callback
     */
    public void addHoldDetector(Detector detector) {
        risingEdgeDetectors.add(detector);
        fallingEdgeDetectors.add(detector);
    }

    /**
     * Add a detector for detecting a button press
     *
     * @param detector The Detector with the watched button and callback
     */
    public void addRisingEdge(Detector detector) {
        risingEdgeDetectors.add(detector);
    }

    /**
     * Add a detector for detecting a button release
     *
     * @param detector The Detector with the watched button and callback
     */
    public void addFallingEdge(Detector detector) {
        fallingEdgeDetectors.add(detector);
    }

    public void processUpdates(Gamepad gamepad) {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad);
        for (Detector detector : risingEdgeDetectors) {
            if (getButton(detector.button, currentGamepad1) && !getButton(detector.button, previousGamepad1)) {
                detector.callback.apply(true);
            }
        }
        for (Detector detector : fallingEdgeDetectors) {
            if (getButton(detector.button, currentGamepad1) && !getButton(detector.button, previousGamepad1)) {
                detector.callback.apply(false);
            }
        }
    }
}
