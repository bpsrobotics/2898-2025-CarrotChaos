package frc.robot.subsystems

import beaverlib.utils.Units.Angular.AngularVelocity
import beaverlib.utils.Units.Angular.radians
import beaverlib.utils.Units.Angular.rotations
import beaverlib.utils.Units.Linear.DistanceUnit
import beaverlib.utils.Units.Linear.VelocityUnit
import beaverlib.utils.Units.Linear.inches
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import frc.robot.RobotMap.FeederSharkId
import frc.robot.RobotMap.IntakeSharkId
import frc.robot.commands.tunnel.StopTunnel
import frc.robot.engine.LaserSharkObjectSensor
import frc.robot.engine.ObjectSensor

class Carrot(var offset: DistanceUnit) {
    companion object {
        val averageLength = 7.25.inches
        val tolerance = 0.45.inches
    }

    val position
        get() = Tunnel.beltPosition - offset

    val inSensorRange: Boolean
        get() = (position < tolerance) && position > averageLength + tolerance
}

object Tunnel : SubsystemBase() {
    private val motor = SparkMax(RobotMap.TunnelId, SparkLowLevel.MotorType.kBrushless)
    private val motorConfig: SparkMaxConfig = SparkMaxConfig()
    private val intakeDetector: ObjectSensor = LaserSharkObjectSensor(IntakeSharkId, 2.inches)
    private val feederDetector: ObjectSensor = LaserSharkObjectSensor(FeederSharkId, 2.inches)

    object CarrotCounter {
        val carrots: MutableList<Carrot> = mutableListOf()

        fun carrotInSensorRange(): Boolean {
            carrots.forEach { carrot -> if (carrot.inSensorRange) return true }
            return false
        }

        fun update() {
            var blockage = Constants.beltLength
            carrots.forEach { carrot ->
                if (carrot.position > blockage) carrot.offset = beltPosition - blockage
                blockage -= Carrot.averageLength
            }
        }

        fun shootCarrot() {
            carrots.removeAt(0)
            carrots.forEach { carrot -> carrot.offset += Carrot.averageLength }
        }
    }

    object Constants {
        val Diameter = 6.inches // todo
        val beltLength = 20.inches
    }

    val beltPosition: DistanceUnit = motor.encoder.position.rotations * Constants.Diameter

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

    override fun periodic() {
        CarrotCounter.update()
//        if (intakeDetector.justEntered() && !CarrotCounter.carrotInSensorRange())
//            CarrotCounter.carrots.add(Carrot(beltPosition))
//        if (feederDetector.justEntered()) CarrotCounter.shootCarrot()
    }
}
