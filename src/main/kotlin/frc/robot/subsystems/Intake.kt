package frc.robot.subsystems

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap

object Intake : SubsystemBase() {
    private val intakeMotor = SparkMax(RobotMap.IntakeId, SparkLowLevel.MotorType.kBrushless)

}