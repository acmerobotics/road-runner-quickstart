package org.firstinspires.ftc.teamcode.robot.swerve;

import androidx.core.math.MathUtils;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SlewRateLimiter {
    private final double m_positiveRateLimit;
    private final double m_negativeRateLimit;
    private final ElapsedTime m_timer;
    private double m_prevVal;
    private double m_prevTime;


    public SlewRateLimiter(double positiveRateLimit, double negativeRateLimit, double initialValue) {
        m_positiveRateLimit = positiveRateLimit;
        m_negativeRateLimit = negativeRateLimit;
        m_prevVal = initialValue;
        m_prevTime = 0;
        m_timer = new ElapsedTime();
    }

    public SlewRateLimiter(double rateLimit, double initalValue) {
        this(rateLimit, -rateLimit, initalValue);
    }

    public SlewRateLimiter(double rateLimit) {
        this(rateLimit, -rateLimit, 0);
    }

    public double calculate(double input) {
        double currentTime = m_timer.seconds();
        double elapsedTime = currentTime - m_prevTime;
        m_prevVal +=
                MathUtils.clamp(
                        input - m_prevVal,
                        m_negativeRateLimit * elapsedTime,
                        m_positiveRateLimit * elapsedTime);
        m_prevTime = currentTime;
        return m_prevVal;
    }
}


