package frc.robot.commands.shooter

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter

class RunGateForTime(val percent: Double, val time: Double = -1.0) : Command() {
    val isInfinite = time < 0
    val timer = Timer()

    init {
        addRequirements(Shooter)
    }

    override fun initialize() {
        if (!isInfinite) timer.restart()
        Shooter.stopShooter()
        Shooter.runGateAtPercent(percent)
    }

    override fun isFinished(): Boolean {
        return !isInfinite && timer.hasElapsed(time)
    }

    override fun end(interrupted: Boolean) {
        Shooter.stop()
    }
}