package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Gamepad;


/**
 * Used to detect cone position, and sleeve color.
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
    public double armManualControl;
    public boolean autoLoadGroundCone;
    public boolean autoLoad34thConeStack;
    public boolean autoLoad45thConeStack;
    public boolean autoLoadThenJunction; // driving robot to high junction after loading cone
    public boolean autoUnloadCone;
    public boolean autoUnloadThenBase; // driving robot to cone loading base after unloading cone
    public boolean teapot; // auto drop off cone, moving to cone base, auto pick up cone, then moving to junction.

    public void checkGamepadButtons(@NonNull Gamepad gamepad1, @NonNull Gamepad gamepad2) {
        //gamepad1 buttons
        robotDrive              = gamepad1.left_stick_y;
        robotStrafe             = gamepad1.left_stick_x;
        robotTurn               = gamepad1.right_stick_x;
        speedDown               = gamepad1.dpad_left || gamepad1.x;
        speedUp                 = gamepad1.dpad_right || gamepad1.b;
        autoLoadGroundCone      = gamepad1.left_bumper;
        autoLoad34thConeStack   = gamepad1.dpad_up;
        autoLoad45thConeStack   = gamepad1.dpad_down;
        autoUnloadCone          = gamepad1.right_bumper;
        autoLoadThenJunction    = gamepad1.left_trigger > 0;
        autoUnloadThenBase      = gamepad1.right_trigger > 0;
        teapot                  = gamepad1.y;

        // gamepad1(single driver) or gamepad2(dual driver) buttons
        sliderUpDown            = gamepad2.right_stick_y;
        sliderWallPosition      = gamepad2.x;
        sliderLowJunction       = gamepad2.a;
        sliderMediumJunction    = gamepad2.b;
        sliderHighJunction      = gamepad2.y;
        sliderGroundJunction    = gamepad2.right_bumper;
        sliderGround            = (gamepad2.right_trigger > 0);

        // gamepad1 or gamepad2
        clawClose               = gamepad2.dpad_up;
        clawOpen                = gamepad2.dpad_down || gamepad1.a;
        if (ArmClawUnit.ArmMode.SWING == ArmClawUnit.armMode) {
            armLeft = gamepad2.left_stick_x < -0.2;
            armRight = gamepad2.left_stick_x > 0.2;
            armForward = gamepad2.left_stick_y > 0.2;
        }

        if (ArmClawUnit.ArmMode.FLIP == ArmClawUnit.armMode) {
            armFrontLoad = gamepad2.left_stick_y < -0.2;
            armBackUnload= gamepad2.left_stick_y > 0.2;
            armManualControl = gamepad2.left_stick_x;
        }
    }

}
