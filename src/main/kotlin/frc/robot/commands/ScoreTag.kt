package frc.robot.commands

import beaverlib.utils.Sugar.clamp
import beaverlib.utils.Units.Angular.degrees
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.Constants
import frc.robot.commands.OI.Rumble
import frc.robot.commands.elevator.MoveElevator
import frc.robot.commands.elevator.StabilizeElevator
import frc.robot.commands.wrist.MoveWrist
import frc.robot.commands.wrist.StabilizeWrist
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Vision
import org.photonvision.targeting.PhotonTrackedTarget

class MoveForward(
    val xPID : PIDController,
    var startPose : Pose2d,
    var distX : () -> Double = {0.0},
    var desiredTag : () -> PhotonTrackedTarget?,
    var subFromDis : Double= 0.0) : Command() {
    override fun execute() {
        xPID.setpoint = distX() - subFromDis

        val xSpeed = xPID.calculate( Drivetrain.pose.x - startPose.x).clamp(-0.1, 0.5)
        println(xSpeed)

        Drivetrain.driveRobotOriented(
            ChassisSpeeds(
                xSpeed, 0.0, 0.0
            )
        )
    }

    override fun isFinished(): Boolean {
        return xPID.error < 0.02
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
    }

}

class ScoreTag(val apriltagId : Int, var yToTag : Double = 0.0) : Command() {
    val xPID = PIDController(1.0, 0.1, 0.1)
    val rotationPID = PIDController(2.0, 0.2, 0.0)
    var hasStarted = false

    init {
        addRequirements(Drivetrain)
        rotationPID.enableContinuousInput(-180.degrees.asRadians, 180.degrees.asRadians)
    }
    var startPose = Pose2d()
    var desiredTag : PhotonTrackedTarget? = null
    override fun initialize() {
        Vision.listeners.add("AlignTag", { result, camera ->
            val desiredTagA = result.targets.filter { it.fiducialId == apriltagId}
            if (desiredTagA.isEmpty() || desiredTagA.first().poseAmbiguity > 0.3) {
                desiredTag = null
                return@add
            }
            desiredTag = desiredTagA.first()
            //val robotToTag = camera.poseEstimator.update(result)
        })
        xPID.setpoint = 1.0
        rotationPID.setpoint = 180.degrees.asRadians
        rotationPID.reset()
        desiredTag = null

        startPose = Drivetrain.pose
        hasStarted = false
    }
    var distX = 0.0
    var command : Command = InstantCommand()
    override fun execute() {
        if(desiredTag != null) distX = desiredTag!!.bestCameraToTarget.x
        if(distX != 0.0 && !command.isScheduled && !hasStarted) {
            hasStarted = true
            command = SequentialCommandGroup(
                ParallelCommandGroup(
                    MoveForward(xPID, startPose, {distX}, {desiredTag}, 1.1),
                    SequentialCommandGroup(MoveWrist(Constants.PivotConstants.PivotState.VerticalBranch.position)),
                    MoveElevator(Constants.ElevatorConstants.ElevatorState.L4.position)),
                ParallelRaceGroup(MoveForward(xPID, startPose, {distX}, {desiredTag}, 0.5), StabilizeWrist(), StabilizeElevator()),
                Rumble(GenericHID.RumbleType.kLeftRumble, 0.5, 0.1)



            )
            command.schedule()
        }

        val xSpeed = xPID.calculate( Drivetrain.pose.x - startPose.x).clamp(-0.1, 0.35)
        println(distX)

        /*Drivetrain.driveRobotOriented(
            ChassisSpeeds(
                xSpeed, 0.0, 0.0
            )
        )*/
    }

    override fun isFinished(): Boolean {
        return hasStarted && command.isFinished
    }
    override fun end(interrupted: Boolean) {
        Vision.listeners.remove("AlignTag")
        command.cancel()
        Drivetrain.stop()
    }
}