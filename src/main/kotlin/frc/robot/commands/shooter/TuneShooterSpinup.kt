package frc.robot.commands.shooter

import beaverlib.utils.Units.Angular.RPM
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Shooter

/** Command that stops the intake motor */
class TuneShooterSpinup() : Command() {
    init {
        addRequirements(Shooter)
        SmartDashboard.putNumber("Shooter/DesiredShooterRPM", 0.0)
    }

    override fun execute() {
        Shooter.stop()
        Shooter.setSpeed(SmartDashboard.getNumber("Shooter/DesiredShooterRPM", 0.0).RPM)
        Shooter.runPIDFF()
    }

    override fun isFinished(): Boolean {
        return false
    }
}
