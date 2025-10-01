package org.riverdell.robotics.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.WebcamName;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectionPipeline;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

@TeleOp(name = "Camera Stream with Detection", group = "FTC")
public class BoxAroundDetection extends LinearOpMode {

    OpenCvWebcam webcam;
    AprilTagDetectionPipeline pipeline;

    // Logitech C270 calibration for 640x480
    double fx = 578.0;
    double fy = 578.0;
    double cx = 320.0;
    double cy = 240.0;
    double tagsize = 0.166; // meters

    @Override
    public void runOpMode() {
        // Webcam setup
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
            .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
            hardwareMap.get(WebcamName.class, "4215"), cameraMonitorViewId);

        pipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy) {
            @Override
            public Mat processFrame(Mat input) {
                Mat output = super.processFrame(input);
                ArrayList<AprilTagDetection> detections = getLatestDetections();

                for (AprilTagDetection tag : detections) {
                    Point p0 = new Point(tag.corners[0].x, tag.corners[0].y);
                    Point p1 = new Point(tag.corners[1].x, tag.corners[1].y);
                    Point p2 = new Point(tag.corners[2].x, tag.corners[2].y);
                    Point p3 = new Point(tag.corners[3].x, tag.corners[3].y);

                    Imgproc.line(output, p0, p1, new Scalar(0, 255, 0), 2);
                    Imgproc.line(output, p1, p2, new Scalar(0, 255, 0), 2);
                    Imgproc.line(output, p2, p3, new Scalar(0, 255, 0), 2);
                    Imgproc.line(output, p3, p0, new Scalar(0, 255, 0), 2);
                }

                return output;
            }
        };

        webcam.setPipeline(pipeline);
        FtcDashboard.getInstance().startCameraStream(webcam, 30);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            ArrayList<AprilTagDetection> detections = pipeline.getLatestDetections();
            TelemetryPacket packet = new TelemetryPacket();

            if (detections.size() > 0) {
                AprilTagDetection tag = detections.get(0);
                packet.put("Tag ID", tag.id);
                packet.put("X (m)", tag.pose.x);
                packet.put("Y (m)", tag.pose.y);
                packet.put("Z (m)", tag.pose.z);
            } else {
                packet.put("Detected", "None");
            }

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            telemetry.update();
            sleep(20);
        }
    }
}
