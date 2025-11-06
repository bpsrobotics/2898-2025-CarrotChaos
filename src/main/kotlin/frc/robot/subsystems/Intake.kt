package frc.robot.subsystems

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import frc.robot.commands.intake.StopIntake

object Intake : SubsystemBase() {
    private val intakeMotor = SparkMax(RobotMap.IntakeId, SparkLowLevel.MotorType.kBrushless)
    private val IntakeConfig: SparkMaxConfig = SparkMaxConfig()

    init {
        // Intake motor initialisation stuff
        IntakeConfig
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(20)

        intakeMotor.configure(
            IntakeConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
        defaultCommand = StopIntake()
    }

    /**
     * Run the intake at the given speed
     * @param percent (-1, 1) the percent speed to run the motor at
     */
    fun runMotor(percent: Double) {
        intakeMotor.set(percent)
    }

    /**
     * Stops the intake motor
     */
    fun stopMotor() {
        intakeMotor.stopMotor()
    }
}