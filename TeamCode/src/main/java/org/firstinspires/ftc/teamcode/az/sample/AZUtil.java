package org.firstinspires.ftc.teamcode.az.sample;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;


public class AZUtil {

    private static final HashMap<String, Object> printMap = new HashMap<>();


    public static void setMotorTargetPosition(DcMotor motor, int pos, double power) {
        motor.setTargetPosition(pos);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }

    public static void setBothMotorTargetPosition(DcMotor motor1, DcMotor motor2,  int pos, double power) {
        motor1.setTargetPosition(pos);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setTargetPosition(pos);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor1.setPower(power);
        motor1.setPower(power);
        motor2.setPower(power);
        motor2.setPower(power);
    }

    public static void waitUntilMotorAtPos(LinearOpMode opMode, DcMotor motor, int pos) {
        int tolerance = 3;
        long currentTimeMs = System.currentTimeMillis();
        while (opMode.opModeIsActive() && motor.isBusy()
                &&
                !(motor.getCurrentPosition() > (pos-3) && motor.getCurrentPosition() < (pos+3))
                && ((System.currentTimeMillis() - currentTimeMs) < 3000))
        {
            opMode.sleep(100);
        }
    }

    public static boolean isAtPos(DcMotor motor, int targetPos) {
        if (motor.getCurrentPosition() == targetPos)
            return true;
        return false;
    }

    static ExecutorService pool = Executors.newFixedThreadPool(1);

    static HashMap<String, ExecutorService> poolMap = new HashMap<>();

    public static final String TURN_TABLE = "TurnTable";
    public static final String FREIGHT_INTAKE_SENSOR = "FreightIntakeSensor";

    static {
        poolMap.put(TURN_TABLE, Executors.newFixedThreadPool(1));
        poolMap.put(FREIGHT_INTAKE_SENSOR, Executors.newFixedThreadPool(1));
    }

    // todo: write your code here

    static Thread thread = null;
    public static void runInParallel(Runnable r) {
//        while (thread != null ){
//            thread.join();
//            // pool.submit(r );
//        }
        thread = new Thread(()->{r.run(); return;});
        thread.start();
    }

    //runs in specified pool. will create pool if does not exist. Recommended that
    //the pool is pre-created ro avoid latency
    public static void runInParallelPool(String poolName, Runnable r) {
        poolMap.get(poolName).execute(r);
    }

    //runs in default pool
    public static void runInParallelPool(Runnable r) {
        pool.execute(()->{r.run(); return;});
    }

    public static void print(Telemetry telemetry, List<Pair<String, Object>> list) {
        list.stream().forEach(p ->printMap.put(p.first, p.second));
        printMapToTelemetry(telemetry);
    }

    private static void printMapToTelemetry(Telemetry telemetry){
        telemetry.addLine(String.valueOf(printMap));
        telemetry.update();
    }
    public static void print(Telemetry telemetry, String str, Object obj) {
        printMap.put(str, obj);
        printMapToTelemetry(telemetry);
//        Set<String> strings = printMap.keySet();
//        Iterator iterator = strings.iterator();
//        while (iterator.hasNext()) {
//            String key = (String) iterator.next();
//            telemetry.addData(key, printMap.get(key));
//
//        }
    }

    public static boolean isMotorAtPosition(DcMotor motor, int pos){
        int currPos = motor.getCurrentPosition();
        return (currPos > pos - 3 && currPos < pos + 3);
    }

    public static Telemetry getMultiTelemetry(LinearOpMode opMode){
        FtcDashboard dashboard = FtcDashboard.getInstance();
        return new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());
    }

    public static void resetMotor(LinearOpMode linearOpMode, DcMotor motor){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitUntilMotorAtPos(linearOpMode, motor, 0);
    }
}