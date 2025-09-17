package frc.robot.commands.swerve


import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.OI
import frc.robot.commands.OI.Rumble
import frc.robot.subsystems.Drivetrain

class NavXReset: Command() {
    private val time = Timer()
    private val swerve: Drivetrain

    init {
        addRequirements(Drivetrain)
        this.swerve = Drivetrain

    }
    override fun initialize() {
        time.reset()
        time.start()
        swerve.zeroGyro()
        Rumble(GenericHID.RumbleType.kRightRumble, 0.25, 1.0).schedule()
    }

    override fun isFinished(): Boolean {
        return time.hasElapsed(0.25)
    }
}