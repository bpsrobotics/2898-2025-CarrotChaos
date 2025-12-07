package frc.robot.commands

import beaverlib.utils.Units.Angular.AngularVelocity
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.commands.intake.RunIntakeForTime
import frc.robot.commands.shooter.OpenloopShooterForTime
import frc.robot.commands.shooter.OpenloopSpinupForTime
import frc.robot.commands.shooter.RunGateForTime
import frc.robot.commands.shooter.ShootForTime
import frc.robot.commands.shooter.SpinupShooter
import frc.robot.commands.tunnel.RunTunnelForTime

fun IntakeForTimeOpenLoop(intakeSpeed: Double, tunnelSpeed: Double, gateReverseSpeed : Double = 0.1, time: Double = -1.0) =
    ParallelRaceGroup(
        RunIntakeForTime(intakeSpeed, time),
        RunTunnelForTime(tunnelSpeed),
        RunGateForTime(-gateReverseSpeed)
    )

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
fun DoShoot(
    shooterSpeed: () -> AngularVelocity,
    gateSpeed: Double,
    tunnelSpeed: Double,
    shootTime: Double = -1.0,
) =
    SequentialCommandGroup(
        SpinupShooter(shooterSpeed, shooterSpeed),
        ParallelRaceGroup(
            ShootForTime(shooterSpeed, gateSpeed),
            RunTunnelForTime(tunnelSpeed),
        ),
    )
fun DoShootIntake(
    shooterSpeed: () -> AngularVelocity,
    intakeSpeed: Double,
    gateSpeed: Double,
    tunnelSpeed: Double,
    shootTime: Double = -1.0,
) =
    SequentialCommandGroup(
        ParallelRaceGroup(
            SpinupShooter(shooterSpeed, shooterSpeed),
            RunIntakeForTime(intakeSpeed,tunnelSpeed)),
        ParallelRaceGroup(

            ShootForTime(shooterSpeed, gateSpeed, shootTime),
            RunIntakeForTime(intakeSpeed),
            RunTunnelForTime(tunnelSpeed),
        ),
    )
fun RunAllRobotForTime(
    intakeSpeed: Double,
    shooterSpeed: Double,
    gateSpeed: Double,
    tunnelSpeed: Double,
    spinupTime: Double,
    shootTime: Double = -1.0,
) =
    SequentialCommandGroup(
        ParallelRaceGroup(
            OpenloopSpinupForTime(shooterSpeed, spinupTime),
            RunIntakeForTime(intakeSpeed,tunnelSpeed)),
        ParallelRaceGroup(

            OpenloopShooterForTime(shooterSpeed, gateSpeed, shootTime),
            RunIntakeForTime(intakeSpeed),
            RunTunnelForTime(tunnelSpeed),
            ),
    )
fun OutakeRobot(
    intakeSpeed: Double,
    shooterSpeed: Double,
    gateSpeed: Double,
    tunnelSpeed: Double,
    time: Double = -1.0,
) =
    SequentialCommandGroup(
        OpenloopShooterForTime(-shooterSpeed, -gateSpeed, time),
            RunIntakeForTime(-intakeSpeed),
            RunTunnelForTime(-tunnelSpeed),
    )
