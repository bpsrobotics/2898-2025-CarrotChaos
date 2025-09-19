package frc.robot.commands

import beaverlib.utils.Units.Angular.asDegrees
import beaverlib.utils.Units.Angular.radians
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Vision
import org.photonvision.targeting.PhotonTrackedTarget
import kotlin.math.absoluteValue
import kotlin.math.sign

class FollowApriltag(val apriltagId : Int) : Command() {
    init {
        addRequirements(Drivetrain)
    }
    var desiredTag : PhotonTrackedTarget? = null
    override fun initialize() {
        Vision.listeners.add("FollowTag", { result, camera ->
            val desiredTagA = result.targets.filter { it.fiducialId == apriltagId}
            if (desiredTagA.isEmpty()) {
                desiredTag = null
                return@add
            }
            desiredTag = desiredTagA.first()
        })
    }

    override fun execute() {
        if(desiredTag == null) { return}
        val yawToTag = desiredTag!!.bestCameraToTarget.rotation.z
        if(yawToTag.radians.asDegrees.absoluteValue > 5)
        Drivetrain.driveRobotOriented(ChassisSpeeds(
            0.0, 0.0, 0.1 * yawToTag.sign
        ))
        val distanceToTag = desiredTag!!.bestCameraToTarget.x
        if(distanceToTag < 1) Drivetrain.driveRobotOriented(ChassisSpeeds(
            0.5, 0.0, 0.0
        ))
    }

    override fun isFinished(): Boolean {
        return false
    }

}