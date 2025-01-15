package org.riverdell.robotics.teleop.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.riverdell.robotics.teleop.tests.config.OuttakePrepareConfig

@TeleOp(
    name = "Prepare Outtake",
    group = "Tests"
)
class PrepareOuttake : LinearOpMode()
{
    override fun runOpMode()
    {
        waitForStart()
        if (isStopRequested)
        {
            return
        }

        val left = hardwareMap["outtakeRotationLeft"] as Servo
        val right = hardwareMap["outtakeRotationRight"] as Servo

        while (opModeIsActive())
        {
            left.position = OuttakePrepareConfig.position
            right.position = 1.0 - OuttakePrepareConfig.position
            Thread.sleep(50L)
        }
    }
}