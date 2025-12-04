package frc.robot.commands

import beaverlib.utils.Units.Angular.RPM
import beaverlib.utils.Units.Angular.asDegrees
import beaverlib.utils.geometry.Vector2
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.Autos
import frc.robot.commands.shooter.ShootForTime
import frc.robot.commands.shooter.SpinupShooter
import frc.robot.commands.vision.AlignOdometry
import frc.robot.engine.FieldMap
import frc.robot.subsystems.Drivetrain
import kotlin.math.sqrt

fun AutoShootCarrots(): Command {
    val targetPose =
        ((Vector2(Drivetrain.pose) - FieldMap.teamFeederStation.center).unit *
                (FieldMap.FeederWidth.asMeters * sqrt(2.0 + 0.1)))
            .toPose2d(Vector2(Drivetrain.pose).angleTo(FieldMap.teamFeederStation.center).asDegrees)
    val desiredShooterSpeed =
        Autos.Constants.shootingPolynomial
            .calculate(Vector2(targetPose).distance(FieldMap.teamFeederStation.center))
            .RPM

    return SequentialCommandGroup(
        ParallelCommandGroup(
            AlignOdometry(targetPose),
            SpinupShooter(desiredShooterSpeed, desiredShooterSpeed),
        ),
        ShootForTime(time = 2.0),
    )
}
