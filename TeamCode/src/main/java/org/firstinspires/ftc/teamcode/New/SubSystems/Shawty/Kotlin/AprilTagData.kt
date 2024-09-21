package org.firstinspires.ftc.teamcode.New.SubSystems.Shawty.Kotlin

import android.util.Size
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.New.SubSystems.SubSystems
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.cos
import kotlin.math.sin

class AprilTagData(hardwareMap: HardwareMap, private val localizer: TeleLocalizer) : SubSystems {

    enum class State {
        On, Off, TagDiscovered
    }

    override var state = State.Off

    private val camera: WebcamName = hardwareMap.get(WebcamName::class.java, "Arducam")

    private var aprilTag: AprilTagProcessor = AprilTagProcessor.Builder()
        .setDrawAxes(true)
        .setDrawTagOutline(true)
        .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
        .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
        .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
        .setLensIntrinsics(919.688, 919.688, 647.849 ,350.162)
        .build()


    private var visionPortal: VisionPortal

    init {
        val builder = VisionPortal.Builder()
            .setCamera(camera)
            .setCameraResolution(Size(1280, 720))
            .enableLiveView(false)
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .addProcessor(aprilTag)

        visionPortal = builder.build()
    }

    fun searchForTag(): Pose2d {
        visionPortal.resumeStreaming()
        visionPortal.resumeLiveView()
        val currentDetections = aprilTag.detections

        for (detection in currentDetections) {
            if (detection.id == 12 || detection.id == 16) {
                state = State.TagDiscovered
                val data = Vector2d(detection.ftcPose.x, detection.ftcPose.y)
                return Pose2d(cameraVector(fieldDistanceToTag(data)), localizer.heading)
            }
        }
        return Pose2d(0.0, 0.0, 0.0)
    }

    private fun fieldDistanceToTag(translateData: Vector2d): Vector2d {
        //todo measure Camera Offset
        val relX = translateData.x + 0.0
        val relY = translateData.y + 1.0
        require(relY > 0)

        val h = -localizer.heading
        val x = relX * cos(h) - relY * sin(h)
        val y = relX * sin(h) + relY * cos(h)

        return Vector2d(x, y)
    }

    private fun cameraVector(fieldDistanceToTag: Vector2d): Vector2d {
        val tagPose = Pair(-72.0, -48.0)
        val xPose: Double = tagPose.first - fieldDistanceToTag.x
        val yPose: Double = tagPose.second - fieldDistanceToTag.y
        return Vector2d(xPose, yPose)
    }

    override fun update() {
        when (state) {
            State.On -> {
                searchForTag()
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
