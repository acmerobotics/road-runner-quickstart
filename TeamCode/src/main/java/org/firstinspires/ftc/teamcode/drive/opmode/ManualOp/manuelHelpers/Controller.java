package org.firstinspires.ftc.teamcode.drive.opmode.helpers;

import com.qualcomm.robotcore.hardware.Gamepad;

/*
    This class defines some useful functions for the controller
    It is basically a wrapper for the generic gamepad access
 */
public class Controller {
    // Create a gamepad object
    private Gamepad gamepad;

    // Create variables to hold the state of each button and analog input
    private int dpad_up, dpad_down, dpad_left, dpad_right;
    private int square, triangle, cross, circle;
    private int left_bumper, right_bumper;

    public double left_stick_x, right_stick_x, left_stick_y, right_stick_y;
    public double left_trigger, right_trigger;

    // This Constructor assigns a given Gamepad object to the gamepad variable
    public Controller(Gamepad g) {
        gamepad = g;
    }

    /*
        This function handles querying the gamepad
        Each time this function is run, it checks the values of all the sticks and buttons
        If a button is pressed, its variable is incremented by 1
        If a button is not pressed, its variable is set to 0
        Analog values are read directly to their respective variables
     */
    public void update() {

        if (gamepad.square) {
            ++square;   // if pressed, increment variable
        } else {
            square = 0; // if not pressed, set to zero
        }
        if (gamepad.triangle) {
            ++triangle;
        } else {
            triangle = 0;
        }
        if (gamepad.cross) {
            ++cross;
        } else {
            cross = 0;
        }
        if (gamepad.circle) {
            ++circle;
        } else {
            circle = 0;
        }
        if (gamepad.dpad_up) {
            ++dpad_up;
        } else {
            dpad_up = 0;
        }
        if (gamepad.dpad_down) {
            ++dpad_down;
        } else {
            dpad_down = 0;
        }
        if (gamepad.dpad_left) {
            ++dpad_left;
        } else {
            dpad_left = 0;
        }
        if (gamepad.dpad_right) {
            ++dpad_right;
        } else {
            dpad_right = 0;
        }
        if (gamepad.left_bumper) {
            ++left_bumper;
        } else {
            left_bumper = 0;
        }
        if (gamepad.right_bumper) {
            ++right_bumper;
        } else {
            right_bumper = 0;
        }

        // This block handles storing the position of the analog inputs as doubles from -1 to 1
        left_stick_x = gamepad.left_stick_x;
        left_stick_y = gamepad.left_stick_y;
        right_stick_x = gamepad.right_stick_x;
        right_stick_y = gamepad.right_stick_y;
        left_trigger = gamepad.left_trigger;
        right_trigger = gamepad.right_trigger;
    }

    /*
        This block of functions provide public boolean wrappers for the private button variables
        If a button's variable is greater than 0, its function will evaluate to true
        If a button's variable is 0, its function will evaluate to false
        These functions are good for querying buttons that will be held for a period of time
     */
    public  boolean dpadUp() {
        return 0 < dpad_up;
    }

    public boolean dpadDown() {
        return 0 < dpad_down;
    }

    public boolean dpadLeft() {
        return 0 < dpad_left;
    }

    public boolean dpadRight() {
        return 0 < dpad_right;
    }

    public boolean Square() {
        return 0 < square;
    }

    public boolean Triangle() {
        return 0 < triangle;
    }

    public boolean Cross() {
        return 0 < cross;
    }

    public boolean Circle() {
        return 0 < circle;
    }

    public boolean leftBumper() {
        return 0 < left_bumper;
    }

    public boolean rightBumper() {
        return 0 < right_bumper;
    }

    /*
        This block of functions provide public boolean wrappers for debounced buttons states
        If a button's variable is equal to 1, its function will evaluate to true
        If a button's variable is 0, its function will evaluate to false
        If a button's variable is greater than 1, its function will evaluate to false
        This ensures that a button is recognized as pressed only 1 time, which is useful for toggles
     */
    public boolean rightTrigger() {return 0 < right_trigger;}

    public boolean leftTrigger() {return  0 < left_trigger;}

    // These functions debounce the button (if the button is held for a long time it will not count
    // as multiple presses)
    // They only return true if the value of a button is equal to 1
    public boolean dpadUpOnce() {
        return 1 == dpad_up;
    }

    public boolean dpadDownOnce() {
        return 1 == dpad_down;
    }

    public boolean dpadLeftOnce() {
        return 1 == dpad_left;
    }

    public boolean dpadRightOnce() {
        return 1 == dpad_right;
    }

    public boolean squareOnce() {
        return 1 == square;
    }

    public boolean triangleOnce() {
        return 1 == triangle;
    }

    public boolean rightTriggerOnce() {return 1 == right_trigger;}

    public boolean leftTriggerOnce() {return 1 == left_trigger;}

    public boolean crossOnce() {
        return 1 == cross;
    }

    public boolean circleOnce() {
        return 1 == circle;
    }

    public boolean leftBumperOnce() {
        return 1 == left_bumper;
    }

    public boolean rightBumperOnce() {
        return 1 == right_bumper;
    }

}