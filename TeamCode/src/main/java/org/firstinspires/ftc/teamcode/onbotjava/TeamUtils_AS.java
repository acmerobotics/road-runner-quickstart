package org.firstinspires.ftc.teamcode.onbotjava;


public class TeamUtils_AS {
    public static double getJoystickAngleInDegrees(double joystickX, double joystickY)
    {//Using -y because forwards is -1
        double angleInRadians = Math.atan2(-joystickY, joystickX);
        double angleInDegrees = angleInRadians * 180 / Math.PI;
       
        //Converting from tangent forward of 90 to robot forward of 0
        angleInDegrees -= 270;//Changed to subtracting 270 instead 90 to invert controls
        if (angleInDegrees < 0)
        {
                angleInDegrees += 360;
        }
        return angleInDegrees;
    }
}
