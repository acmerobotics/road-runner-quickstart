package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.subsystems.settings.GamepadSettings;

public class InputModification {
    /**
     * Returns the powered input
     * @param base base input
     * @return base to the EXPONENT_MODIFIER power
     */
    public static double poweredInput(double base, int modifier) {
        if (modifier % 2 == 0) {
            return Math.pow(base, GamepadSettings.EXPONENT_MODIFIER) * Math.signum(base);
        } else {
            return Math.pow(base, GamepadSettings.EXPONENT_MODIFIER);
        }
    }
}
