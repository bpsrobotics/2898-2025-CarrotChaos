package frc.robot.subsystems

import beaverlib.utils.Units.Angular.AngularVelocity
import beaverlib.utils.Units.Angular.radians
import beaverlib.utils.Units.Linear.VelocityUnit
import beaverlib.utils.Units.Linear.inches
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import frc.robot.commands.tunnel.StopTunnel

object Tunnel : SubsystemBase() {
    private val motor = SparkMax(RobotMap.TunnelId, SparkLowLevel.MotorType.kBrushless)
    private val motorConfig: SparkMaxConfig = SparkMaxConfig()

    object Constants {
        val Diameter = 6.inches // todo
    }

    init {
        // Intake motor initialisation stuff
        motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(20)

        motor.configure(
            motorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters,
        )
        defaultCommand = StopTunnel()
    }

    fun runAtPercent(percent: Double) {
        motor.set(percent)
    }

    fun stop() {
        motor.stopMotor()
    }

    fun runAtSpeed(speed: AngularVelocity) {
        // todo
    }

    fun runAtSpeed(speed: VelocityUnit) {
        runAtSpeed((speed / Constants.Diameter) * 1.radians)
    }
}
