package org.firstinspires.ftc.teamcode.util.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.hardware.Component;

public class StepperServo extends Component {

    private float angle;
    public Servo servo;
    public AnalogInput encoder;

    public StepperServo(int port, String name, HardwareMap map){
        super(port, name);
        servo = map.servo.get(name);
    }

    public StepperServo(int port, String name, HardwareMap map, String encoder){
        super(port, name);
        servo = map.servo.get(name);
        this.encoder = map.get(AnalogInput.class, encoder);
    }

    public void setAngle(float angle) {
        this.angle = angle;
        servo.setPosition(angle/355);
    }

    public void addAngle(float angle) {
        this.angle += angle % 360;
        servo.setPosition(angle/355);
    }

    public float getAngle(){
        return (float) servo.getPosition();
    }
    public float getAngle(boolean useEncoder) {
        return (float) (encoder.getVoltage() / 3.3 * 360);
    }

    public float getAngleDeg() {
        return angle;
    }

}