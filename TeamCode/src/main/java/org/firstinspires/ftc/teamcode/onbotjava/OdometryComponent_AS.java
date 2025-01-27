package org.firstinspires.ftc.teamcode.onbotjava;
import static com.qualcomm.robotcore.util.TypeConversion.byteArrayToInt;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.Locale;
import com.qualcomm.robotcore.robot.Robot;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


public class OdometryComponent_AS extends RobotComponent_AS {
    //write code here
    GoBildaPinpointDriver_AS odo;
    Pose2D currentPosition;
    Pose2D currentVelocity;
    
    public OdometryComponent_AS(Robot2024_AS _robot){
        super(_robot);
         // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        IllegalArgumentException e=null;
        
        for (int i=0; i<10; i++) {
            try {
                odo = hardwareMap.get(GoBildaPinpointDriver_AS.class,"odo");
                break;
            } catch (IllegalArgumentException e1) {
                e=e1;
                odo = (GoBildaPinpointDriver_AS) hardwareMap.get("odo");
                robot.alert("Problem with odo, retrying");
                try {Thread.sleep(500);} catch (InterruptedException ee){return;}
            }
        }
        if ( odo==null )
            throw e;

        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        odo.setOffsets(-85.0, -170.0); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver_AS.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver_AS.EncoderDirection.REVERSED, GoBildaPinpointDriver_AS.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        odo.recalibrateIMU();
        odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES,0));
        odo.getVelocity();
        loop();
    }

    public void loop(){
        odo.update();
        currentPosition = odo.getPosition();
        currentVelocity = odo.getVelocity();
    }
    public void doTelemetry(Telemetry telemetry){
        String data = String.format(Locale.US, "{X: %.1f in, Y: %.1f in, H: %.1f}", 
            currentPosition.getX(DistanceUnit.INCH), currentPosition.getY(DistanceUnit.INCH), 
            currentPosition.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Odo position", data);
    }
    
    public void setPosition(Pose2D newPosition) {
        odo.setPosition(newPosition);
    }
}    


    
    
    
    
