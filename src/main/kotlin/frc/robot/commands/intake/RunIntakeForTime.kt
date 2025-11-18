package frc.robot.commands.intake

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Intake

class RunIntakeForTime(val percent: Double, val time: Double = -1.0) : Command() {
    val isInfinite = time < 0
    val timer = Timer()

    init {
        addRequirements(Intake)
    }

    override fun initialize() {
        if (!isInfinite) timer.restart()
        Intake.runMotor(percent)
    }

    override fun isFinished(): Boolean {
        return !isInfinite && timer.hasElapsed(time)
    }

    override fun end(interrupted: Boolean) {
        Intake.runMotor(0.0)
    }
}
