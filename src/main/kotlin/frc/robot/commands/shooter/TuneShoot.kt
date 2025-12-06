package frc.robot.commands.shooter

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Shooter

/**
 * Runs the Shooter at the current [Shooter.setSpeed] values for the given time (indefinitely if
 * [time] is negative])
 *
 * @param gatePercent the speed to run the [Shooter.gateMotor] at
 * @param time The time to run the shooter for (indefinite if negative)
 */
class TuneShoot(val gatePercent: Double = 0.6) : Command() {
    init {
        addRequirements(Shooter)
    }

    override fun initialize() {
        Shooter.runGateAtPercent(gatePercent)
    }

    override fun execute() {
        Shooter.runPIDFF()
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        Shooter.stop()
    }
}
