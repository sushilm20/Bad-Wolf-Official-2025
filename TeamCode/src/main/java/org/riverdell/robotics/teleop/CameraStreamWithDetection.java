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

import java.util.ArrayList;

@TeleOp(name = "Camera Stream with Detection", group = "FTC")
public class CameraStreamWithDetection extends LinearOpMode {

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

        pipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        webcam.setPipeline(pipeline);

        // Start camera stream to Dashboard
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
