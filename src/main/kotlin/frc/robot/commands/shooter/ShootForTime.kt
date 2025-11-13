package frc.robot.commands.shooter

import beaverlib.utils.Units.Angular.AngularVelocity
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Shooter

/**
 * Runs the Shooter at the current [Shooter.setSpeed] values for the given time (indefinitely if [time] is negative])
 * @param gatePercent the speed to run the [Shooter.gateMotor] at
 * @param time The time to run the shooter for (indefinite if negative)
 */
class ShootForTime (val gatePercent : Double = 0.1, val time : Double = -1.0) : Command() {
    init { addRequirements(Shooter) }
    val isInfinite = time < 0
    val timer = Timer()

    override fun initialize() {
        if(!isInfinite) timer.restart()
        Shooter.runGateAtPercent(gatePercent)
    }

    override fun execute() {
        Shooter.runPIDFF()
    }

    override fun isFinished(): Boolean {
        return !isInfinite && timer.hasElapsed(time)
    }

    override fun end(interrupted: Boolean) {
        Shooter.runAtPercent(0.0)
    }
}