package frc.robot.commands.tunnel

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Tunnel

/**
 * Command that stops the intake motor
 */
class StopTunnel() : Command() {
    init { addRequirements(Tunnel) }

    override fun execute() { Tunnel.stop() }

    override fun isFinished(): Boolean { return false }
}