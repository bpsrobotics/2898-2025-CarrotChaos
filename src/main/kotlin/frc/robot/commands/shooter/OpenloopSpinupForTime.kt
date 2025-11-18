package frc.robot.commands.shooter

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Shooter

/**
 * Runs both shooter motors at the given [percent] for [time], without running the
 * [Shooter.gateMotor], in order to ensure the shooter is spinning at a fast speed before carrots
 * are fed
 *
 * @param percent The power to run the shooter motors at
 * @param time The amount of time to run the shooter for
 */
class OpenloopSpinupForTime(val percent: Double, val time: Double) : Command() {
    val timer = Timer()

    init {
        addRequirements(Shooter)
    }

    override fun initialize() {
        timer.restart()
        Shooter.runAtPercent(percent)
        Shooter.stopGate()
    }

    override fun isFinished(): Boolean {
        return timer.hasElapsed(time)
    }

    override fun end(interrupted: Boolean) {
        Shooter.runAtPercent(0.0)
    }
}
