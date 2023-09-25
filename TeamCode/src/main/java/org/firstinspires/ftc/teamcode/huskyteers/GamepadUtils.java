package org.firstinspires.ftc.teamcode.huskyteers;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.function.Consumer;

public class GamepadUtils {
    private static class Detector {
        Consumer<Boolean> callback;
        String button;

        Detector(String button, Consumer<Boolean> callback) {
            this.callback = callback;
            this.button = button;
        }
    }

    private final Gamepad currentGamepad1 = new Gamepad();
    private final Gamepad previousGamepad1 = new Gamepad();

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
     * @param button   The button to watch
     * @param callback The callback to the function when pressed or released
     */
    public void addHoldDetector(String button, Consumer<Boolean> callback) {
        risingEdgeDetectors.add(new Detector(button, callback));
        fallingEdgeDetectors.add(new Detector(button, callback));
    }

    /**
     * Add a detector for detecting a button press
     *
     * @param button   The button to watch
     * @param callback The callback to the function when pressed
     */
    public void addRisingEdge(String button, Consumer<Boolean> callback) {
        risingEdgeDetectors.add(new Detector(button, callback));
    }

    /**
     * Add a detector for detecting a button release
     *
     * @param button   The button to watch
     * @param callback The callback to the function when released
     */
    public void addFallingEdge(String button, Consumer<Boolean> callback) {
        fallingEdgeDetectors.add(new Detector(button, callback));
    }

    public void processUpdates(Gamepad gamepad) {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad);
        for (Detector detector : risingEdgeDetectors) {
            if (getButton(detector.button, currentGamepad1) && !getButton(detector.button, previousGamepad1)) {
                detector.callback.accept(true);
            }
        }
        for (Detector detector : fallingEdgeDetectors) {
            if (getButton(detector.button, currentGamepad1) && !getButton(detector.button, previousGamepad1)) {
                detector.callback.accept(false);
            }
        }
    }
}
