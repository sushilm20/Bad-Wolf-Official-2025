package org.riverdell.robotics.subsystems

import com.arcrobotics.ftclib.drivebase.MecanumDrive
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.autonomous.movement.localization.TwoWheelLocalizer

class Drivetrain(private val robot: HypnoticRobot) : AbstractSubsystem()
{
    private val voltageSensor = robot.opMode.hardwareMap.voltageSensor.first()
    private val voltageState by state(write = { _ -> }, read = { voltageSensor.voltage })

    val localizer by lazy {
        TwoWheelLocalizer(robot)
    }

    private lateinit var backingDriveBase: MecanumDrive

    fun voltage() = kotlin.runCatching { voltageState.current() }.getOrElse { 0.0 }
    fun imu() = robot.imuProxy.imu()

    fun driveRobotCentric(driverOp: GamepadEx, scaleFactor: Double)
    {
        backingDriveBase.driveRobotCentric(
            -driverOp.leftX * scaleFactor,
            -driverOp.leftY * scaleFactor,
            -driverOp.rightX * scaleFactor,
            true
        )
    }


    override fun start()
    {

    }

    /**
     * Initializes both the IMU and all drivebase motors.
     */
    override fun doInitialize()
    {
        if (robot.opMode is HypnoticAuto)
        {
            robot.hardware.frontLeft.direction = DcMotorSimple.Direction.FORWARD
            robot.hardware.frontLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

            robot.hardware.frontRight.direction = DcMotorSimple.Direction.REVERSE
            robot.hardware.frontRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

            robot.hardware.backLeft.direction = DcMotorSimple.Direction.FORWARD
            robot.hardware.backLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

            robot.hardware.backRight.direction = DcMotorSimple.Direction.REVERSE
            robot.hardware.backRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

            runWithoutEncoders()
        } else
        {
            setupDriveBase()
        }
    }

    fun setupDriveBase()
    {
        val backLeft = Motor(robot.opMode.hardwareMap, "backLeft")
        val backRight = Motor(robot.opMode.hardwareMap, "backRight")
        val frontLeft = Motor(robot.opMode.hardwareMap, "frontLeft")
        val frontRight = Motor(robot.opMode.hardwareMap, "frontRight")

        backingDriveBase = MecanumDrive(
            frontLeft, frontRight, backLeft, backRight
        )
    }

    fun stopAndResetMotors() = configureMotorsToDo {
        it.power = 0.0
        it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
    }

    fun runWithoutEncoders() = configureMotorsToDo {
        it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    private fun configureMotorsToDo(consumer: (DcMotor) -> Unit)
    {
        listOf(robot.hardware.backLeft, robot.hardware.frontLeft, robot.hardware.frontRight, robot.hardware.backRight).forEach(consumer::invoke)
    }

    override fun isCompleted() = true
    override fun dispose()
    {
        if (robot.opMode is HypnoticAuto)
        {
            stopAndResetMotors()
        }
    }
}