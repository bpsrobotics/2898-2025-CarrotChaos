package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Intake

/** Command that stops the intake motor */
class StopIntake() : Command() {
    init {
        addRequirements(Intake)
    }

    override fun execute() {
        Intake.stopMotor()
    }

    override fun isFinished(): Boolean {
        return false
    }
}
