package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Opmodes.Testing$Tuning.Subsystems$Tele.Attachment.PID;

//TODO(change into one class??)

/*
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled
@TeleOp(name="PIDforDrive", group="Linear OpMode")
public class PIDforDrive extends LinearOpMode {

    public static double[] p = new double[4];
    public static double[] i = new double[4];
    public static double[] d = new double[4];

    //in inches
    public static int target=0;
    public double ENCtoINC =(9.6/2.54* Math.PI)/537.689839572;


    @Override
    public void runOpMode() {

        p[0] = .006;
        i[0] = 0;
        d[0] = .0001;

        PIDController RF = new PIDController(p[0], i[0], d[0]);


        /*
        PIDController RB = new PIDController(p,i, d);


        PIDController LF = new PIDController(p, i, d);

        PIDController Lb = new PIDController(p, i, d);



import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while(opModeIsActive()) {

            RF.setPID(p[0], i[0], d[0]);

            //inches
            double pose = ENCtoINC * rightFront.getCurrentPosition();

            double power = RF.calculate(pose, target);

            rightFront.setPower(power);

            telemetry.addData("pose", pose);
            telemetry.addData("target",target);

            telemetry.update();

        }


    }
}
*/
