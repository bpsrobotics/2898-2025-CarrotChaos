package frc.robot.subsystems

import beaverlib.utils.Units.Angular.AngularVelocity
import beaverlib.utils.Units.Angular.radians
import beaverlib.utils.Units.Linear.VelocityUnit
import beaverlib.utils.Units.Linear.inches
import beaverlib.utils.Units.Linear.meters
import beaverlib.utils.Units.Linear.metersPerSecond
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import frc.robot.commands.intake.StopIntake
import beaverlib.utils.Units.*
import frc.robot.commands.shooter.StopShooter

object Shooter : SubsystemBase() {
    private val motorTop = SparkMax(RobotMap.IntakeId, SparkLowLevel.MotorType.kBrushless)
    private val motorBottom = SparkMax(RobotMap.IntakeId, SparkLowLevel.MotorType.kBrushless)
    private val gateMotor = SparkMax(RobotMap.IntakeId, SparkLowLevel.MotorType.kBrushless)


    private val shooterConfig : SparkMaxConfig = SparkMaxConfig()
    private val gateConfig : SparkMaxConfig = SparkMaxConfig()

    object Constants {
        val WheelRadius =  6.inches //todo
    }

    init {
        // Intake motor initialisation stuff
        shooterConfig
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(20)

        motorTop.configure(
            shooterConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
        motorBottom.configure(
            shooterConfig.inverted(true),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )

        // Intake motor initialisation stuff
        gateConfig
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(20)
        gateMotor.configure(
            gateConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )

        defaultCommand = StopShooter()
    }

    fun runAtPercent(percent : Double) {
        motorTop.set(percent)
        motorBottom.set(percent)
    }
    fun runGateAtPercent(percent : Double) {
        gateMotor.set(percent)
    }

    fun stop() {
        motorTop.stopMotor()
        motorBottom.stopMotor()
        gateMotor.stopMotor()
    }
    fun stopShooter() {
        motorTop.stopMotor()
        motorBottom.stopMotor()
    }
    fun stopGate() {
        gateMotor.stopMotor()
    }

    fun runAtSpeed(speed : AngularVelocity) {
        //todo
    }
    fun runAtSpeed(speed : VelocityUnit) { runAtSpeed((speed / Constants.WheelRadius) * 1.radians) }
}
