package org.riverdell.robotics.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectionPipeline;

import java.util.ArrayList;

@TeleOp(name = "AprilTag Follower", group = "Linear OpMode")
public class AprilTagFollower extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private OpenCvWebcam webcam;
    private AprilTagDetectionPipeline pipeline;

    // Camera calibration values
    double fx = 578.0;       // Focal length in pixels (x-axis)
    double fy = 578.0;       // Focal length in pixels (y-axis)
    double cx = 320.0;       // Optical center x (image center)
    double cy = 240.0;       // Optical center y (image center)
    double tagsize = 0.166;  // Tag size in meters (adjust to your actual tag size)


    // Control constants
    double kP = 0.5;
    double centerTolerance = 0.02;

    @Override
    public void runOpMode() {
        // Initialize drivetrain motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Initialize webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
            .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
            hardwareMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, "4215"),
            cameraMonitorViewId);

        pipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        webcam.setPipeline(pipeline);

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

            if (detections.size() > 0) {
                AprilTagDetection tag = detections.get(0);
                double xOffset = tag.pose.x;

                telemetry.addData("Tag ID", tag.id);
                telemetry.addData("X Offset", xOffset);

                if (Math.abs(xOffset) > centerTolerance) {
                    double strafePower = kP * xOffset;
                    strafe(strafePower);
                } else {
                    stopMotors();
                }
            } else {
                telemetry.addLine("No tag detected");
                stopMotors();
            }

            telemetry.update();
            sleep(20);
        }
    }

    private void strafe(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(-power);
        frontRight.setPower(-power);
        backRight.setPower(power);
    }

    private void stopMotors() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }
}
