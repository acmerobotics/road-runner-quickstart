package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public final class StickyGamepad
{
    private final OpMode opMode;
    private final Gamepad gamepad;

    public boolean a = false, b = false, x = false, y = false, ps = false;
    public boolean dpad_left = false, dpad_up = false, dpad_right = false, dpad_down = false;
    public boolean left_bumper = false, right_bumper = false;
    public boolean left_stick_button = false, right_stick_button = false;

    private double unstickAfter = 121;

    private double a_down = -122, b_down = -122, x_down = -122, y_down = -122, ps_down = -122;
    private double dpad_left_down = -122, dpad_up_down = -122, dpad_right_down = -122, dpad_down_down = -122;
    private double left_bumper_down = -122, right_bumper_down = -122;
    private double left_stick_button_down = -122, right_stick_button_down = -122;

    public StickyGamepad(Gamepad gamepad, OpMode opMode)
    {
        this.gamepad = gamepad;
        this.opMode = opMode;
    }

    public void update()
    {
        if (gamepad.a)
        {
            if (opMode.getRuntime() < a_down + unstickAfter)
            {
                a = false;
            }
            else
            {
                a_down = opMode.getRuntime();
                a = true;
            }
        }
        else
        {
            a = false;
            a_down = -122;
        }

        if (gamepad.b)
        {
            if (opMode.getRuntime() < b_down + unstickAfter)
            {
                b = false;
            }
            else
            {
                b_down = opMode.getRuntime();
                b = true;
            }
        }
        else
        {
            b = false;
            b_down = -122;
        }

        if (gamepad.x)
        {
            if (opMode.getRuntime() < x_down + unstickAfter)
            {
                x = false;
            }
            else
            {
                x_down = opMode.getRuntime();
                x = true;
            }
        }
        else
        {
            x = false;
            x_down = -122;
        }

        if (gamepad.y)
        {
            if (opMode.getRuntime() < y_down + unstickAfter)
            {
                y = false;
            }
            else
            {
                y_down = opMode.getRuntime();
                y = true;
            }
        }
        else
        {
            y = false;
            y_down = -122;
        }

        if (gamepad.ps)
        {
            if (opMode.getRuntime() < ps_down + unstickAfter)
            {
                ps = false;
            }
            else
            {
                ps_down = opMode.getRuntime();
                ps = true;
            }
        }
        else
        {
            ps = false;
            ps_down = -122;
        }

        if (gamepad.dpad_left)
        {
            if (opMode.getRuntime() < dpad_left_down + unstickAfter)
            {
                dpad_left = false;
            }
            else
            {
                dpad_left_down = opMode.getRuntime();
                dpad_left = true;
            }
        }
        else
        {
            dpad_left = false;
            dpad_left_down = -122;
        }

        if (gamepad.dpad_up)
        {
            if (opMode.getRuntime() < dpad_up_down + unstickAfter)
            {
                dpad_up = false;
            }
            else
            {
                dpad_up_down = opMode.getRuntime();
                dpad_up = true;
            }
        }
        else
        {
            dpad_up = false;
            dpad_up_down = -122;
        }

        if (gamepad.dpad_right)
        {
            if (opMode.getRuntime() < dpad_right_down + unstickAfter)
            {
                dpad_right = false;
            }
            else
            {
                dpad_right_down = opMode.getRuntime();
                dpad_right = true;
            }
        }
        else
        {
            dpad_right = false;
            dpad_right_down = -122;
        }

        if (gamepad.dpad_down)
        {
            if (opMode.getRuntime() < dpad_down_down + unstickAfter)
            {
                dpad_down = false;
            }
            else
            {
                dpad_down_down = opMode.getRuntime();
                dpad_down = true;
            }
        }
        else
        {
            dpad_down = false;
            dpad_down_down = -122;
        }

        if (gamepad.left_bumper)
        {
            if (opMode.getRuntime() < left_bumper_down + unstickAfter)
            {
                left_bumper = false;
            }
            else
            {
                left_bumper_down = opMode.getRuntime();
                left_bumper = true;
            }
        }
        else
        {
            left_bumper = false;
            left_bumper_down = -122;
        }

        if (gamepad.right_bumper)
        {
            if (opMode.getRuntime() < right_bumper_down + unstickAfter)
            {
                right_bumper = false;
            }
            else
            {
                right_bumper_down = opMode.getRuntime();
                right_bumper = true;
            }
        }
        else
        {
            right_bumper = false;
            right_bumper_down = -122;
        }

        if (gamepad.left_stick_button)
        {
            if (opMode.getRuntime() < left_stick_button_down + unstickAfter)
            {
                left_stick_button = false;
            }
            else
            {
                left_stick_button_down = opMode.getRuntime();
                left_stick_button = true;
            }
        }
        else
        {
            left_stick_button = false;
            left_stick_button_down = -122;
        }

        if (gamepad.right_stick_button)
        {
            if (opMode.getRuntime() < right_stick_button_down + unstickAfter)
            {
                right_stick_button = false;
            }
            else
            {
                right_stick_button_down = opMode.getRuntime();
                right_stick_button = true;
            }
        }
        else
        {
            right_stick_button = false;
            right_stick_button_down = -122;
        }
    }

    public void unstickAfter(double sec) {
        unstickAfter = sec;
    }

    public void dontUnstick() {
        unstickAfter(121);
    }
}
