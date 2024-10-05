package org.firstinspires.ftc.teamcode.util.hardware;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.hardware.Component;

import java.util.concurrent.TimeUnit;

public class GoBildaLEDIndicator extends Component {
    public Servo led;
    float color = 0;
    Animation currentAnimation = Animation.SOLID;

    private ElapsedTime elaspedTimeBlink = new ElapsedTime();
    private boolean blinkStat = false;


    public GoBildaLEDIndicator(int port, String name, HardwareMap map) {
        super(port, name);
        led = map.get(Servo.class, name);
    }

    public void setColor(Colors c) {
        switch (c) {
            case OFF:
                color = 0;
                break;
            case RED:
                color = (float) 0.277;
                break;
            case ORANGE:
                color = (float) 0.333;
                break;
            case JOOS_ORANGE:
                color = (float) 0.334;
                break;
            case YELLOW:
                color = (float) 0.388;
                break;
            case SAGE:
                color = (float) 0.444;
                break;
            case GREEN:
                color = (float) 0.5;
                break;
            case AZURE:
                color = (float) 0.555;
                break;
            case BLUE:
                color = (float) 0.611;
                break;
            case INDIGO:
                color = (float) 0.666;
                break;
            case VIOLET:
                color = (float) 0.722;
                break;
            case WHITE:
                color = (float) 1;
                break;
        }
    }

    public void setColor(float pwmSignal) {
        color = pwmSignal;
    }

    public void setColor(int r, int g, int b) {
        float[] hsv = new float[3];
        Color.RGBToHSV(r, g, b, hsv);

        color = (float) ((hsv[0] / (275/0.445)) + 0.277);
    }

    public void setAnimation(Animation a) {
        currentAnimation = a;
    }

    public void update() {
        if (currentAnimation == Animation.SOLID) {
            led.setPosition(color);
        } else if (currentAnimation == Animation.BLINK) {
            if (elaspedTimeBlink.time(TimeUnit.MILLISECONDS) >= 250) {
                if (blinkStat) {
                    led.setPosition(0);
                    blinkStat = !blinkStat;
                    elaspedTimeBlink.reset();
                } else {
                    led.setPosition(color);
                    blinkStat = !blinkStat;
                    elaspedTimeBlink.reset();
                }
            }
        } else if (currentAnimation == Animation.SLOW_BLINK) {
            if (elaspedTimeBlink.time(TimeUnit.MILLISECONDS) >= 500) {
                if (blinkStat) {
                    led.setPosition(0);
                    blinkStat = !blinkStat;
                    elaspedTimeBlink.reset();
                } else {
                    led.setPosition(color);
                    blinkStat = !blinkStat;
                    elaspedTimeBlink.reset();
                }
            }
        }
    }

    public enum Colors {
        OFF,
        RED,
        ORANGE,
        JOOS_ORANGE,
        YELLOW,
        SAGE,
        GREEN,
        AZURE,
        BLUE,
        INDIGO,
        VIOLET,
        WHITE
    }

    public enum Animation {
        SOLID,
        BLINK,
        SLOW_BLINK
    }
}
