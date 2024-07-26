package org.firstinspires.ftc.teamcode.BacktrackingKt

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime

@TeleOp(name = "Backtracking_TuningKt", group = "Linear OpMode")
class BacktrackingTUNING : LinearOpMode() {
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val drive = Drive(hardwareMap, Pose2d(0.0, 0.0, 0.0))

        val loopTimer = ElapsedTime()
        loopTimer.reset()

        while (opModeInInit()) {
            telemetry.addLine("The DIRECTIONS")
            telemetry.addLine("Tune variable in FTC Dashboard under Backtracking_Tuning")
            telemetry.addLine("Watch and tune for best localizer accuracy")
            telemetry.addLine("Turn and push your robot for about 30 seconds")
            telemetry.addLine("To make it more realistic simulate robot to robot contact")
            telemetry.update()
        }

        telemetry.clearAll()

        var numOfLoops=1
        var total =0
        waitForStart()

        while (opModeIsActive()) {

            total+= loopTimer.milliseconds().toInt()

            //reset timer to show more accurate average loop times
            if(total> 5000){
                total =0
                numOfLoops=0
            }

            loopTimer.reset()
            drive.updatePoseEstimate()

            //Weird X is Y things are because Y in RR is lateral movement
            telemetry.addData("Overall Effectiveness X - In Percentage", drive.totalMovement()[0])
            telemetry.addData("Overall Effectiveness Y - In Percentage", drive.totalMovement()[1])

            telemetry.addData("Overall Error in inches (X)", MecDrive.OverallError[0])
            telemetry.addData("Overall Error in inches (Y)", MecDrive.OverallError[1])
            telemetry.addData("Overall Error in degrees (R)", Math.toDegrees(MecDrive.OverallError[2]))

            telemetry.addData("Rotation - Pose", Math.toDegrees(drive.pose.heading.toDouble()))
            telemetry.addData("X - Pose", drive.pose.position.y)
            telemetry.addData("Y - Pose", drive.pose.position.x)

            telemetry.addData("LoopTime", loopTimer.milliseconds().toInt())

            if(numOfLoops!=0){
                telemetry.addData("Avg LoopTime" , (total/numOfLoops))
            }

            telemetry.update()

            numOfLoops++
        }
    }
    @Config
    object Backtracking_TuningKt{
        //You can tune the time at which the program reads the IMU to find best accuracy
        @JvmField var timeBetween_Reads: Int = 300
    }
}
