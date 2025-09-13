// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
@file:Suppress("unused")

package frc.robot

import beaverlib.utils.Units.Linear.feet
import beaverlib.utils.Units.Linear.feetPerSecond
import beaverlib.utils.Units.Linear.inches
import beaverlib.utils.Units.lb
import com.pathplanner.lib.config.ModuleConfig
import com.pathplanner.lib.config.RobotConfig
import beaverlib.utils.Units.Electrical.Current
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.util.Color
import frc.robot.Constants.DriveConstants.DriveKinematics
import frc.robot.Constants.DriveConstants.MaxSpeedMetersPerSecond
import java.io.File
import kotlin.math.PI

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 *
 *
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
@Suppress("MemberVisibilityCanBePrivate")
class Constants {
    object DriveConstants {
        val MaxSpeedMetersPerSecond = (3.1).feetPerSecond.asMetersPerSecond
        // Chassis configuration (left to right dist of center of the wheels)
        val TrackWidth = Units.inchesToMeters(11.5)

        // Distance between centers of right and left wheels on robot (front to back dist)
        val WheelBase = Units.inchesToMeters(11.5)

        // Distance between front and back wheels on robot: CHANGE TO MATCH WITH ROBOT
        val DriveKinematics = arrayOf(
            Translation2d(WheelBase / 2, TrackWidth / 2),
            Translation2d(WheelBase / 2, -TrackWidth / 2),
            Translation2d(-WheelBase / 2, TrackWidth / 2),
            Translation2d(-WheelBase / 2, -TrackWidth / 2)
        )
        // YAGSL `File` Configs
        val DRIVE_CONFIG: File = File(Filesystem.getDeployDirectory(), "swerve1")

        val MomentOfInertia = 4.09149392  // kg * m^2
    }

    object OIConstants {
        const val DriverControllerPort = 0
        @Suppress("SpellCheckingInspection")
        const val DriveDeadband = 0.05
        const val SpeedMultiplierMin = 0.4
        const val SpeedMultiplierMax = 1.0
        const val DEADZONE_THRESHOLD = 0.1
    }

    object AutoConstants {
        val Robot_Config = RobotConfig(
            (120.0).lb.asKilograms,
            MaxSpeedMetersPerSecond,
            ModuleConfig(
                (2.0).inches.asMeters,
                MaxSpeedMetersPerSecond,
                1.54,
                DCMotor.getNEO(1).withReduction(6.75),
                30.0,
                1
            ),
            *DriveKinematics

        )
        const val MaxAccelerationMetersPerSecondSquared = 3.0
        const val MaxAngularSpeedRadiansPerSecond = Math.PI
        const val MaxAngularSpeedRadiansPerSecondSquared = Math.PI
        const val PXController = 1.0
        const val PYController = 1.0
        const val PThetaController = 1.0

        const val TranslationP = 5.0
        const val TranslationI = 0.0
        const val TranslationD = 0.0

        const val RotationP = 0.01
        const val RotationI = 0.0
        const val RotationD = 0.0

        // Constraint for the motion profiled robot angle controller
        val ThetaControllerConstraints = TrapezoidProfile.Constraints(
                MaxAngularSpeedRadiansPerSecond, MaxAngularSpeedRadiansPerSecondSquared
        )
    }

    // set to operator/driver's preferences
    object ButtonConstants {
        //Driver buttons
        const val RESET_GYRO = 6

        //Operator Controls
    }

    object VisionConstants {
        const val CORAL_OFFSET_FROM_CENTER = 0.1524
    }
}