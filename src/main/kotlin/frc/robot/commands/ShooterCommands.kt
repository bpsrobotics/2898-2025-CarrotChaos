package frc.robot.commands

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.commands.intake.RunIntakeForTime
import frc.robot.commands.shooter.OpenloopShooterForTime
import frc.robot.commands.shooter.OpenloopSpinupForTime
import frc.robot.commands.tunnel.RunTunnelForTime

fun IntakeForTimeOpenLoop(intakeSpeed: Double, tunnelSpeed: Double, time: Double = -1.0) =
    ParallelRaceGroup(RunIntakeForTime(intakeSpeed, time), RunTunnelForTime(tunnelSpeed))

fun ShootForTimeOpenLoop(
    shooterSpeed: Double,
    gateSpeed: Double,
    tunnelSpeed: Double,
    spinupTime: Double,
    shootTime: Double = -1.0,
) =
    SequentialCommandGroup(
        OpenloopSpinupForTime(shooterSpeed, spinupTime),
        ParallelRaceGroup(
            OpenloopShooterForTime(shooterSpeed, gateSpeed, shootTime),
            RunTunnelForTime(tunnelSpeed),
        ),
    )
