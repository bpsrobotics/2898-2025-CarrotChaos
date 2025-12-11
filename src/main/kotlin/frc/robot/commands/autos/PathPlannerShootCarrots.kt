package frc.robot.commands.autos

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.Autos

class PathPlannerShootCarrots() : Command() {
    var command: Command = InstantCommand()

    override fun initialize() {
        command =
            Autos.pathFindToPose(AutoShootCarrots.targetPose)
                .alongWith(AutoShootCarrots.spinup)
                .andThen(AutoShootCarrots.shoot)
        command.schedule()
    }

    override fun isFinished(): Boolean {
        return command.isFinished
    }

    override fun end(interrupted: Boolean) {
        command.cancel()
    }
    // Create the desired pose given the rotation
    // Autos.Constants.shootingPolynomial.calculate(distanceToFeeder.asMeters).RPM
}
