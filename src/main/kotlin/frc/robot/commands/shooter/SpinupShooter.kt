package frc.robot.commands.shooter

import beaverlib.utils.Units.Angular.AngularVelocity
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.commands.ShootForTimeOpenLoop
import frc.robot.subsystems.Shooter

/**
 * Runs the shooter until both [Shooter.motorBottom] and [Shooter.motorTop] are at the desired [bottomSpeed] and [topSpeed] respectivly
 * @param bottomSpeed Desired speed of the bottomMotor
 * @param topSpeed Desired speed of the topMotor
 */
class SpinupShooter (val bottomSpeed : AngularVelocity, val topSpeed : AngularVelocity) : Command() {
    init { addRequirements(Shooter) }

    override fun initialize() {
        Shooter.setSpeeds(bottomSpeed, topSpeed)
        Shooter.stopGate()
    }

    override fun execute() {
        Shooter.runPIDFF()
    }

    override fun isFinished(): Boolean {
        return Shooter.isAtSpeed()
    }

    override fun end(interrupted: Boolean) {
        Shooter.runAtPercent(0.0)
    }
}