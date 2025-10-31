// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

//import com.team2898.robot.Constants.OperatorConstants

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import frc.robot.OI.translationX
import frc.robot.OI.translationY
import frc.robot.OI.turnX
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.OI.configureBindings
import frc.robot.OI.resetGyro
import frc.robot.subsystems.Drivetrain
import frc.robot.commands.swerve.TeleopDriveCommand
import frc.robot.commands.OI.NavXReset
import kotlin.math.pow
import kotlin.math.sign

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
    // The robot's subsystems and commands are defined here...
    //private val m_exampleSubsystem = ExampleSubsystem()
    // Replace with CommandPS4Controller or CommandJoystick if needed

    private var autoCommandChooser: SendableChooser<Command> = SendableChooser()
    val alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red)

    val reverseDrive = if(alliance == DriverStation.Alliance.Red) {-1.0} else {1.0}

    val teleopDrive: TeleopDriveCommand =
        TeleopDriveCommand(
            { MathUtil.applyDeadband(translationY*reverseDrive, 0.1) },
            { MathUtil.applyDeadband(translationX*reverseDrive, 0.1) },
            { MathUtil.applyDeadband(-turnX, 0.1)},
            { true },
            { 0.25 }
        )






    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
        initializeObjects()

        Drivetrain.defaultCommand = teleopDrive

        configureBindings()
    }
    fun getAutonomousCommand(): Command{
        val path = autoCommandChooser.selected
        return path
    }

    private fun initializeObjects() {
        Drivetrain
    }
}