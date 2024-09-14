package org.firstinspires.ftc.teamcode.New.SubSystems.Kotlin

import android.util.Size
import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.sin

class AprilTagData(hardwareMap: HardwareMap, private val localizer: TeleLocalizer): SubSystems {

    enum class State {
        On, Off, TagDiscovered
    }

    override var state = State.Off

    private val camera: WebcamName = hardwareMap.get(WebcamName::class.java, "Webcam1")
    private var aprilTag: AprilTagProcessor
    private var visionPortal: VisionPortal

    init {
        val builder: VisionPortal.Builder = VisionPortal.Builder()
        builder.setCamera(camera)
        builder.setCameraResolution(Size(1920, 1080))
        aprilTag = AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawTagOutline(true)
            .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
            .build()


        //todo change when testing
        builder.enableLiveView(false)

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG)

        // Set and enable the processor.
        builder.addProcessor(aprilTag)

        visionPortal = builder.build()
    }

    private fun searchForTag(): Pose2d {
        visionPortal.resumeStreaming()
        val currentDetections= aprilTag.detections
        val data = doubleArrayOf(0.0,0.0)
        for (detection in currentDetections) {
            if (detection.id == 2 || detection.id == 4) {
                data[0] = detection.ftcPose.range
                data[1] = detection.ftcPose.bearing
            }
        }
        return findCameraPose(data)
    }

    private fun findCameraPose(translateData: DoubleArray): Pose2d {
        val Range = translateData[0]
        val Bearing = translateData[1]

        val yaw = if (localizer.heading >= 0) {
            90 - localizer.heading
        } else {
            90 + localizer.heading
        }

        val x = Range * sin(Math.PI / 2 - Bearing) / sin(Math.PI / 2 + yaw)
        val y = Range * sin(Bearing - yaw) / sin(Math.PI / 2 + yaw)

        return Pose2d(x,y,localizer.heading)
    }

    override fun update() {
        when(state){
            State.On -> {
                searchForTag()
                if(searchForTag() != Pose2d(0.0,0.0,0.0)) state = State.TagDiscovered
            }
            State.Off -> {
                visionPortal.stopStreaming()
            }
            State.TagDiscovered -> {
                val pose = searchForTag()
                //run to pose
                //if Pose reached
                state = State.Off
            }
        }
    }

}