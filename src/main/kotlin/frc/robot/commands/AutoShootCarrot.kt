package frc.robot.commands.autos

import beaverlib.utils.Units.Angular.RPM
import beaverlib.utils.Units.Angular.asDegrees
import beaverlib.utils.Units.seconds
import beaverlib.utils.geometry.vector2
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.DeferredCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup
import frc.robot.commands.autos.AutoShootCarrotsStuff.shoot
import frc.robot.commands.autos.AutoShootCarrotsStuff.spinup
import frc.robot.commands.autos.AutoShootCarrotsStuff.targetPose
import frc.robot.commands.vision.AlignOdometry
import frc.robot.engine.FieldMap
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Gate
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Tunnel
import kotlin.math.sqrt

fun AutoShootCarrots() =
    DeferredCommand(
        { AlignOdometry(targetPose).alongWith(spinup).andThen(shoot) },
        setOf(Drivetrain, Shooter, Tunnel, Gate),
    )

object AutoShootCarrotsStuff {

    val targetPose: Pose2d
        get() =
            ((Drivetrain.pose.vector2 - FieldMap.teamFeederStation.center).unit *
                    (FieldMap.FeederWidth * sqrt(2.0 + 0.1) + Drivetrain.Constants.BumperWidth / 2)
                        .asMeters) // Get the closest pose that is proper distance from the
                // feeder
                .toPose2d(
                    Drivetrain.pose.vector2.angleTo(FieldMap.teamFeederStation.center).asDegrees -
                        180
                )

    val desiredShooterSpeed = { 3500.RPM }

    val spinup = Shooter.spinup(desiredShooterSpeed)
    val shoot =
        ParallelRaceGroup(
            Shooter.shoot(desiredShooterSpeed, time = 2.0.seconds),
            Gate.runAtPowerCommand(0.4),
        )
}

var command: Command = InstantCommand()
