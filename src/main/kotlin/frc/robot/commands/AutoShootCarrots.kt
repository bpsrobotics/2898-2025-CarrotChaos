package frc.robot.commands

import beaverlib.utils.Units.Angular.RPM
import beaverlib.utils.Units.Angular.asDegrees
import beaverlib.utils.Units.Linear.meters
import beaverlib.utils.Units.seconds
import beaverlib.utils.geometry.Vector2
import beaverlib.utils.geometry.vector2
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.commands.vision.AlignOdometry
import frc.robot.engine.FieldMap
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Gate
import frc.robot.subsystems.Shooter
import kotlin.math.sqrt

fun AutoShootCarrots(): Command {
    val zooRadius = FieldMap.zooSafeRadius(Drivetrain.Constants.BumperWidth / 2)
    val distanceToFeeder = {
        Drivetrain.pose.vector2.distance(FieldMap.teamFeederStation.center).meters
    }

    // Gets pose that puts robot in range of the feeder
    val targetPose = {
        ((Drivetrain.pose.vector2 - FieldMap.teamFeederStation.center).unit *
                (FieldMap.FeederWidth * sqrt(2.0 + 0.1) + Drivetrain.Constants.BumperWidth / 2)
                    .asMeters) // Get the closest pose that is proper distance from the feeder
            .toPose2d(Vector2(Drivetrain.pose).angleTo(FieldMap.teamFeederStation.center).asDegrees)
    }
    // Create the desired pose given the rotation
    val desiredShooterSpeed = { 3500.RPM }
    // Autos.Constants.shootingPolynomial.calculate(distanceToFeeder.asMeters).RPM

    return SequentialCommandGroup(
        ParallelCommandGroup(AlignOdometry(targetPose), Shooter.spinup(desiredShooterSpeed)),
        ParallelRaceGroup(
            Shooter.shoot(desiredShooterSpeed, time = 2.0.seconds),
            Gate.runAtPowerCommand(0.4),
        ),
    )
}
