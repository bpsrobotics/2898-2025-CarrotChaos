package frc.robot.commands.shooter

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter

/**
 * Runs the Shooter at the given [percent] values using openloop for the given time (indefinitely if
 * [time] is negative])
 *
 * @param percent The speed to run the shooter motors at
 * @param gatePercent the speed to run the [Shooter.gateMotor] at
 * @param time The time to run the shooter for (indefinite if negative)
 */
class OpenloopShooterForTime(
    val percent: Double,
    val gatePercent: Double,
    val time: Double = -1.0,
) : Command() {
    val isInfinite = time < 0
    val timer = Timer()

    init {
        addRequirements(Shooter)
    }

    override fun initialize() {
        if (!isInfinite) timer.restart()
        Shooter.runAtPercent(percent)
        Shooter.runGateAtPercent(gatePercent)
    }

    override fun isFinished(): Boolean {
        return !isInfinite && timer.hasElapsed(time)
    }

    override fun end(interrupted: Boolean) {
        Intake.runMotor(0.0)
    }
}
