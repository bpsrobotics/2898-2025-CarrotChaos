package frc.robot.commands.vision

import beaverlib.utils.Sugar.clamp
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.sign

// 3, 3.2
class AlignOdometry(
    var targetPose2d: Pose2d,
    val maxSpeed: Double = 1.0,
    val maxRotSpeed: Double = 0.8,
) : Command() {
    companion object {
        val deadzone = 0.05
        val ks = 0.05
        val yPID = PIDController(3.0, 0.3, 0.1)
        val xPID = PIDController(3.0, 0.3, 0.1)
        val rotationPID = PIDController(3.0, 0.2, 0.1)

        init {
            rotationPID.enableContinuousInput(-PI, PI)
        }
    }

    init {
        addRequirements(Drivetrain)
    }

    override fun initialize() {
        xPID.reset()
        yPID.reset()
        rotationPID.reset()

        rotationPID.setpoint = MathUtil.angleModulus(targetPose2d.rotation.radians)
        xPID.setpoint = targetPose2d.x
        yPID.setpoint = targetPose2d.y
        println(targetPose2d)
    }

    override fun execute() {
        var rotationSpeed = rotationPID.calculate(Drivetrain.pose.rotation.radians)
        var xSpeed = xPID.calculate(Drivetrain.pose.x)
        var ySpeed = yPID.calculate(Drivetrain.pose.y)

        if (xSpeed.absoluteValue < deadzone) xSpeed = 0.0 else xSpeed += ks * xSpeed.sign
        //
        if (ySpeed.absoluteValue < deadzone) ySpeed = 0.0 else ySpeed += ks * ySpeed.sign
        //
        if (rotationSpeed.absoluteValue < deadzone) rotationSpeed = 0.0
        else rotationSpeed += ks * rotationSpeed.sign

        val totalError = rotationPID.error + xPID.error + yPID.error

        if (totalError < 0.2) {
            when {
                xPID.error > yPID.error && xPID.error > rotationPID.error * 2 -> {
                    ySpeed = 0.0
                    rotationSpeed = 0.0
                }
                yPID.error > xPID.error && yPID.error > rotationPID.error * 2 -> {
                    xSpeed = 0.0
                    rotationSpeed = 0.0
                }
                else -> {
                    xSpeed = 0.0
                    ySpeed = 0.0
                }
            }
        }

        Drivetrain.driveFieldOriented(
            ChassisSpeeds(
                xSpeed.clamp(-maxSpeed, maxSpeed),
                ySpeed.clamp(-maxSpeed, maxSpeed),
                rotationSpeed.clamp(-maxRotSpeed, maxRotSpeed),
            )
        )
    }

    override fun isFinished(): Boolean {
        return rotationPID.error < 0.01 && xPID.error < 0.01 && yPID.error < 0.01
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
    }
}
