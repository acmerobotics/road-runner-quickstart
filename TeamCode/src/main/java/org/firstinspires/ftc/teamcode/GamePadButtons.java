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
    public float sliderUpDown;
    public boolean speedDown;
    public boolean speedUp;
    public boolean sliderWallPosition;
    public boolean sliderGroundJunction;
    public boolean sliderGround;
    public boolean sliderLowJunction;
    public boolean sliderMediumJunction;
    public boolean sliderHighJunction;
    public boolean clawClose;
    public boolean clawOpen;
    public boolean armLeft;
    public boolean armRight;
    public boolean armForward;
    public boolean armFrontLoad;
    public boolean armFrontUnload;
    public boolean armBackLoad;
    public boolean armBackUnload;
    public int armManualControl;
    public boolean autoLoadGroundCone;
    public boolean autoLoad34thConeStack;
    public boolean autoLoad45thConeStack;
    public boolean autoLoadThenJunction; // driving robot to high junction after loading cone
    public boolean autoUnloadCone;
    public boolean autoUnloadThenBase; // driving robot to cone loading base after unloading cone
    public boolean teapot; // auto drop off cone, moving to cone base, auto pick up cone, then moving to junction.
    public boolean backBase; // back to cone base when starting Teleop

    public void checkGamepadButtons(@NonNull Gamepad gamepad1, @NonNull Gamepad gamepad2) {
        //gamepad1 buttons
        robotDrive = gamepad1.left_stick_y;
        robotStrafe = gamepad1.left_stick_x;
        robotTurn = gamepad1.right_stick_x;
        speedDown = gamepad1.dpad_left || gamepad1.x;
        speedUp = gamepad1.dpad_right || gamepad1.b;
        autoLoadGroundCone = gamepad1.left_bumper;
        autoLoad34thConeStack = gamepad1.dpad_up;
        autoLoad45thConeStack = gamepad1.dpad_down;
        autoUnloadCone = gamepad1.right_bumper;
        autoLoadThenJunction = gamepad1.left_trigger > 0;
        autoUnloadThenBase = gamepad1.right_trigger > 0;
        teapot = gamepad1.y;
        backBase = gamepad1.back;

        // gamepad1 and gamepad2
        clawOpen = gamepad2.dpad_down || gamepad1.a;

        // gamepad2 buttons
        clawClose = gamepad2.dpad_up;

        armManualControl = (int) (gamepad2.left_stick_y * 40);
    }
}
