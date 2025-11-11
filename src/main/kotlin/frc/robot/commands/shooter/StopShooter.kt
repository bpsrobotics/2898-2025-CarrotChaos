package frc.robot.commands.shooter

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter

/**
 * Command that stops the intake motor
 */
class StopShooter() : Command() {
    init { addRequirements(Shooter) }

    override fun execute() {
        Shooter.stop()

    }

    override fun isFinished(): Boolean { return false }
}