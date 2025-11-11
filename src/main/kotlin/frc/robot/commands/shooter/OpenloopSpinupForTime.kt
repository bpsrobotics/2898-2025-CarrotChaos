package frc.robot.commands.shooter

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter

class OpenloopSpinupForTime(val percent : Double, val time : Double) : Command() {
    val timer = Timer()
    init { addRequirements(Shooter) }

    override fun initialize() {
        timer.restart()
        Shooter.runAtPercent(percent)
    }

    override fun isFinished(): Boolean {
        return timer.hasElapsed(time)
    }

    override fun end(interrupted: Boolean) {
        Shooter.runAtPercent(0.0)
    }
}