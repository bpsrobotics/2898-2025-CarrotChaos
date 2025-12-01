package frc.robot.engine

import com.revrobotics.spark.SparkMax
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.MutAngle
import edu.wpi.first.units.measure.MutAngularVelocity
import edu.wpi.first.units.measure.MutVoltage
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog

class SysIdMotorLogger(val name: String, val motor: SparkMax) {
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private val m_appliedVoltage: MutVoltage = Volts.mutable(0.0)
    private val m_angle: MutAngle = Radians.mutable(0.0)
    private val m_velocity: MutAngularVelocity = RadiansPerSecond.mutable(0.0)

    fun log(log: SysIdRoutineLog) {
        log.motor(name)
            .voltage(
                m_appliedVoltage.mut_replace(
                    motor.get() * RobotController.getBatteryVoltage(),
                    Volts,
                )
            )
            .angularPosition(m_angle.mut_replace(motor.encoder.position, Rotations))
            .angularVelocity(m_velocity.mut_replace(motor.encoder.velocity, RotationsPerSecond))
    }
}
