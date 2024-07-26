package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Tele.AutoDrive

import com.qualcomm.robotcore.hardware.HardwareMap

class Camera(hardwareMap: HardwareMap?) {
/*
    init{
        Companion.hardwareMap = hardwareMap
    }

    companion object {
        private var hardwareMap: HardwareMap? = null
        val camera: WebcamName = hardwareMap!!.get(WebcamName::class.java, "Webcam1")
        private var aprilTag: AprilTagProcessor? = null
        var visionPortal: VisionPortal? = null

        var Detection: Boolean = false
        var newRange: Double = 0.0
        var Xchange: Double = 0.0
        var Ychange: Double = 0.0

        private val shoulderAttachment = arrayOf(13.0, Shoulder.shoulderAngle())
        private val wristAttachment = arrayOf(7.9, Wr.wristAngle)
        private val attachmentList = arrayOf(shoulderAttachment, wristAttachment)

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


            // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
            builder.enableLiveView(true)

            // Set the stream format; MJPEG uses less bandwidth than default YUY2.
            builder.setStreamFormat(VisionPortal.StreamFormat.YUY2)

            // Set and enable the processor.
            builder.addProcessor(aprilTag)


            visionPortal = builder.build()
        }

        private fun translateCamData(translateData: DoubleArray): DoubleArray {
            val Yaw: Double
            val Range = translateData[1]
            val Bearing = translateData[2]
            if (Drive.rx!! >= 0) {
                Yaw = 90 - Drive.rx!!
            } else {
                Yaw = 90 + Drive.rx!!
            }

            Xchange = Range * sin(Math.PI / 2 - Bearing) / sin(Math.PI / 2 + Yaw)
            Ychange = Range * sin(Bearing - Yaw) / sin(Math.PI / 2 + Yaw)

            //change range so that turn is updated to new board offset
            //List the attachments being considered when checking DFB

            newRange = hypot(Xchange - DistanceFromBoard.distanceFromTAG(attachmentList), Ychange)

            // test for inches per degree: InchMoved/360
            val turnCoefficient = 0.0
            val turnNeeded = turnCoefficient * Yaw
            //To have turning be done right before it reaches the board.
            val turnSpeed = turnNeeded / newRange

            //New Range Used to get New travel direction
            val xChange = newRange * sin(90 - Bearing) / sin(90 + Yaw)
            val yChange = -1 * newRange * sin(Bearing - Yaw) / sin(90 + Yaw)

            val translateCamData = DoubleArray(4)
            translateCamData[0] = xChange
            translateCamData[1] = yChange
            translateCamData[2] = turnSpeed
            translateCamData[3] = newRange
            return translateCamData
        }

        fun detectAprilTag(): DoubleArray {
            Detection = false
            val currentDetections: List<AprilTagDetection> = aprilTag!!.detections
            var CamData = DoubleArray(3)
            for (detection in currentDetections) {
                if (detection.metadata != null && (detection.id == 2 || detection.id == 4)) {
                    Detection = true
                    val translate = DoubleArray(3)
                    translate[1] = detection.ftcPose.range
                    translate[2] = detection.ftcPose.bearing

                    //translate camera data
                    CamData = translateCamData(translate)
                }
            }
            return CamData
        }
        /*
        fun DetectAprilTagRawData(): DoubleArray {
            Detection = false
            val currentDetections: List<AprilTagDetection> = aprilTag.getDetections()
            val CamData = doubleArrayOf(0.0, 0.0, 0.0)
            for (detection in currentDetections) {
                if (detection.metadata != null && (detection.id == 2 || detection.id == 4)) {
                    CamData[1] = detection.ftcPose.range
                    CamData[2] = detection.ftcPose.bearing
                    Detection = true
                }
            }
            return CamData
        }

         */

    }

 */


}
