package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.WebcamName;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectionPipeline;

import java.util.ArrayList;

@TeleOp(name = "AprilTag Follower", group = "FTC")
public class AprilTagFollowerOpMode extends LinearOpMode {

    OpenCvWebcam webcam;
    AprilTagDetectionPipeline pipeline;

    DcMotor frontLeft, frontRight, backLeft, backRight;

    // Camera calibration values
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166; // meters

    // Control constants
    double kP = 0.5; // Proportional gain
    double centerTolerance = 0.02; // meters

    @Override
    public void runOpMode() {
        // Motors
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
            .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
            hardwareMap.get(WebcamName.class, "4215"), cameraMonitorViewId);

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
                AprilTagDetection tag = detections.get(0); // Use first tag
                double xOffset = tag.pose.x; // Horizontal offset in meters

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

    // Strafes robot left/right based on power (+ = right, - = left)
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
