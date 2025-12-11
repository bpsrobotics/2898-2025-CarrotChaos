package frc.robot.commands.autos

import beaverlib.utils.Units.Linear.DistanceUnit
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.commands.vision.AlignOdometry
import frc.robot.subsystems.Drivetrain

class AutoShootMove(val wait: Double, val y: DistanceUnit, val x: DistanceUnit) : Command() {
    companion object {
        val autoShoot = AutoShootCarrots()
    }

    var command: Command = InstantCommand()

    override fun initialize() {
        command =
            autoShoot
                .andThen(WaitCommand(wait))
                .andThen(AlignOdometry(Pose2d(Drivetrain.pose.x, y.asMeters, Rotation2d())))
                .andThen(AlignOdometry(Pose2d(x.asMeters, Drivetrain.pose.x, Rotation2d())))
        command.schedule()
    }

    override fun isFinished(): Boolean {
        return command.isFinished
    }

    override fun end(interrupted: Boolean) {
        command.cancel()
    }
}
