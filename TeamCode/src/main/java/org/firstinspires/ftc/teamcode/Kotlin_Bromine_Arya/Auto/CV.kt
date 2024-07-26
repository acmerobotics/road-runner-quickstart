package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto

import android.graphics.Canvas
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Rect
import org.opencv.imgproc.Imgproc

class CV : VisionProcessor {

    // Three possible locations
    lateinit var Left_Rect: Mat
    lateinit var Middle_Rect: Mat
    lateinit var Right_Rect: Mat
    private var CameraHeight = 1920
    private var CameraWidth = 1080

    // HSV MAT
    val HSV_Mat = Mat()

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {
        CameraHeight = height

        Left_Rect = HSV_Mat.submat(makeRectangle(1))
        Middle_Rect = HSV_Mat.submat(makeRectangle(2))
        Right_Rect = HSV_Mat.submat(makeRectangle(3))
    }

    override fun processFrame(frame: Mat?, captureTimeNanos: Long) {
        Imgproc.cvtColor(frame, HSV_Mat, Imgproc.COLOR_RGB2HSV)

        val avg1 = Core.mean(Left_Rect).`val`[0].toInt().toDouble()
        val avg2 = Core.mean(Middle_Rect).`val`[0].toInt().toDouble()
        val avg3 = Core.mean(Right_Rect).`val`[0].toInt().toDouble()

        val maxOf12 = Math.max(avg1,avg2)
        val max = Math.max(maxOf12,avg3)

        when(max){
            avg1 ->location = Location.Left
            avg2 ->location = Location.Middle
            avg3 ->location = Location.Right
        }

    }


    override fun onDrawFrame(
        canvas: Canvas?,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?
    ) {
        TODO("Not yet implemented")
    }

    fun makeRectangle (region: Int): Rect {
        return Rect(Point(region/3.0*CameraWidth, CameraHeight.toDouble()),Point((region+1)/3.0*CameraWidth,
            CameraHeight.toDouble()
        ))
    }
    private var location: Location = Location.None

    fun TeamPropLocation(): Location {
        return location
    }
    enum class Location{
        Left, Right, Middle, None
    }

}