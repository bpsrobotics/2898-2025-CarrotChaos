package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import frc.robot.Constants.AutoConstants.RotationD
import frc.robot.Constants.AutoConstants.RotationI
import frc.robot.Constants.AutoConstants.RotationP
import frc.robot.Constants.AutoConstants.TranslationD
import frc.robot.Constants.AutoConstants.TranslationI
import frc.robot.Constants.AutoConstants.TranslationP
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Drivetrain.driveConsumer
import frc.robot.subsystems.Drivetrain.getAlliance

object Autos {

    init {
        AutoBuilder.configure(
            { Drivetrain.pose },  // Robot pose supplier
            Drivetrain::resetOdometry,  // Method to reset odometry (will be called if your auto has a starting pose)
            { Drivetrain.robotVelocity },  // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            driveConsumer,  // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            PPHolonomicDriveController( // PPolonomicController is the built-in path following controller for holonomic drive trains
                PIDConstants(TranslationP, TranslationI, TranslationD),  // Translation PID constants
                PIDConstants(RotationP, RotationI, RotationD)
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


}