package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Gamepad;


/**
 * Game pad buttons design:
 * Game Pad1:
 *  Left stick:
 *      left-right: strafe robot
 *      up-down:    drive root
 *  Right stick:
 *      left-right: turn robot
 *  dpad:
 *      left:       robot movement speed down
 *      right:      robot movement speed up
 *      up:         normal auto pickup the 3rd cone from cone stack
 *      down:       normal auto pickup the 4th and 5th cone from cone stack
 *  X:              robot movement speed down (same as dpad_left)
 *  B:              robot movement speed down (same as dpad_right)
 *  Left Bumper:    normal auto pickup cone from ground
 *  Right bumper:   normal auto drop off cone
 *  Left trigger:   auto pick up cone from cone base then move to high junction, slider lifted to high junction
 *  Right trigger:  auto drop off cone on high junction then move to cone base
 *  Y:              Teapot function - drop off cone on high junction then pick up cone from base, then move to high junction
 *  Back:           Back to cone base just at the beginning of Teleop
 *  A:              open claw to drop off cone manually.
 *
 * Game pad2:
 *  dpad:
 *      Up:         close the claw to pick up cone
 *      Down:       open claw to drop off cone manually
 *  Right stick:
 *                  left-right: manually control slider up and down
 *  X:              move slider to wall position
 *  A:              move slider to low junction position
 *  B:              move slider to medium junction position
 *  Y:              move slider to high junction position
 *  Right bumper:   move slider to ground junction position
 *  Right trigger:  move slider to ground position
 *  Left stick:
 *      left-right: manually control arm position
 *      up:         move arm to pick up position
 *      down:       move arm to drop off position
 *
 */
public class GamePadButtons {
    //game pad setting
    public float robotDrive;
    public float robotStrafe;
    public float robotTurn;
    public boolean speedDown;
    public boolean speedUp;
    public boolean armLift;
    public boolean armDown;

    public boolean wristUp;
    public boolean wristDown;

    public boolean fingerOuttake;
    public boolean fingerIntake;
    public boolean fingerStop;

    public boolean switchOpen;
    public boolean switchClose;

    public boolean accelorate;

    public boolean readyToIntake;
    public boolean dropPosition;
    public boolean launchOn;

    public void checkGamepadButtons(@NonNull Gamepad gamepad1, @NonNull Gamepad gamepad2) {
        //gamepad1 buttons
        robotDrive = gamepad1.left_stick_y;
        robotStrafe = gamepad1.left_stick_x;
        robotTurn = gamepad1.right_stick_x;

        // arm lift
        armLift = gamepad1.x;
        armDown = gamepad1.y;

        // launch plane
        launchOn = gamepad1.right_bumper || gamepad2.right_bumper;

        fingerIntake = gamepad1.dpad_down || gamepad2.dpad_down;
        fingerStop = gamepad1.dpad_left || gamepad2.dpad_left;
        fingerOuttake = gamepad1.dpad_up || gamepad2.dpad_up;;

        wristUp = gamepad1.a;
        wristDown = gamepad1.b;

        switchOpen = (gamepad1.right_trigger > 0);
        switchClose = (gamepad1.left_trigger > 0);

        readyToIntake = gamepad1.left_bumper;

        dropPosition = gamepad1.right_bumper;

        accelorate = gamepad1.back;

    }
}
