package frc.robot

import beaverlib.controls.PathPlannerPID
import beaverlib.utils.Units.Angular.AngularAcceleration
import beaverlib.utils.Units.Angular.AngularVelocity
import beaverlib.utils.Units.Angular.radiansPerSecond
import beaverlib.utils.Units.Angular.radiansPerSecondSquared
import beaverlib.utils.Units.Linear.Acceleration
import beaverlib.utils.Units.Linear.VelocityUnit
import beaverlib.utils.Units.Linear.metersPerSecond
import beaverlib.utils.Units.Linear.metersPerSecondSquared
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.Constants.AutoConstants.TranslationPIDConstant
import frc.robot.Constants.AutoConstants.RotationPIDConstant
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Drivetrain.driveConsumer
import frc.robot.subsystems.Drivetrain.getAlliance
import kotlin.math.PI

object Autos {

    init {
        AutoBuilder.configure(
            { Drivetrain.pose },  // Robot pose supplier
            Drivetrain::resetOdometry,  // Method to reset odometry (will be called if your auto has a starting pose)
            { Drivetrain.robotVelocity },  // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            driveConsumer,  // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            PPHolonomicDriveController( // PPolonomicController is the built-in path following controller for holonomic drive trains
                TranslationPIDConstant.PathPlannerPID,  // Translation PID constants
                RotationPIDConstant.PathPlannerPID
            ),
            Constants.AutoConstants.Robot_Config,
            getAlliance,
            Drivetrain// Reference to this subsystem to set requirements
        )
    }
    /**
     * Gets a command that follows a path created in PathPlanner.
     * @param pathName The path's file name.
     * @param setOdomAtStart Whether to update the robot's odometry to the start pose of the path.
     * @return A command that follows the path.
     */
//    fun getAutonomousCommand(
//        autoName: String,
//       setOdomAtStart: Boolean
//    ): Command {
//        var startPosition: Pose2d = Pose2d()
//        if(PathPlannerAuto.getStaringPoseFromAutoFile(autoName) == null) {
//            startPosition = PathPlannerAuto.getPathGroupFromAutoFile(autoName)[0].startingDifferentialPose
//        } else {
//            startPosition = PathPlannerAuto.getStaringPoseFromAutoFile(autoName)
//        }
//
//        if(DriverStation.getAlliance() == Optional.of(Alliance.Red)){
//            startPosition = GeometryUtil.flipFieldPose(startPosition)
//        }
//
//        if (setOdomAtStart)
//        {
//            if (startPosition != null) {
//                resetOdometry(startPosition)
//            }
//        }
//
//        // TODO: Configure path planner's AutoBuilder
//        return PathPlannerAuto(autoName)
//    }
    fun generatePath(vararg pose2dWaypoints: Pose2d, maxVelocity : VelocityUnit = 3.0.metersPerSecond, maxAcceleration : Acceleration = 3.0.metersPerSecondSquared,
                     maxAngularVelocity : AngularVelocity = (2*PI).radiansPerSecond, maxAngularAcceleration: AngularAcceleration = (4* PI).radiansPerSecondSquared ): PathPlannerPath {
        // Create a list of waypoints from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
        val waypoints = PathPlannerPath.waypointsFromPoses(
            pose2dWaypoints.asList()
        )

        val constraints = PathConstraints(
            maxVelocity.asMetersPerSecond,
            maxAcceleration.asMetersPerSecondSquared,
            maxAngularVelocity.asRadiansPerSecond,
            maxAngularAcceleration.asRadiansPerSecondSquared
        ) // The constraints for this path.
        // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

        // Create the path using the waypoints created above
        val path = PathPlannerPath(
            waypoints,
            constraints,
            null,  // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
            GoalEndState(
                0.0,
                Rotation2d.fromDegrees(-90.0)
            ) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        )
        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true
        return path

    }


}