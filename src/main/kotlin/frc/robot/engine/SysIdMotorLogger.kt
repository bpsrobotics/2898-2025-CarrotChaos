package frc.robot.engine

import beaverlib.utils.Units.Time
import beaverlib.utils.Units.seconds
import com.revrobotics.spark.SparkMax
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.MutAngle
import edu.wpi.first.units.measure.MutAngularVelocity
import edu.wpi.first.units.measure.MutVoltage
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism

class BeaverSysIDRoutine(subsystem: SubsystemBase, vararg motors: SysidMotor) {
    val routine =
        SysIdRoutine(
            SysIdRoutine.Config(),
            Mechanism(
                { volts -> motors.forEach { motor -> motor.setVoltage(volts) } },
                { log ->
                    // Record a frame for the shooter motor.
                    motors.forEach { motor -> motor.log(log) }
                },
                subsystem,
            ),
        )

    /**
     * Returns a command that will execute a quasistatic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    fun sysIdQuasistatic(direction: SysIdRoutine.Direction?): Command {
        return routine.quasistatic(direction)
    }

    /**
     * Returns a command that will execute a dynamic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    fun sysIdDynamic(direction: SysIdRoutine.Direction?): Command {
        return routine.dynamic(direction)
    }

    fun fullSysID(waitTime: Time = 3.0.seconds): Command {
        return routine
            .dynamic(SysIdRoutine.Direction.kForward)
            .andThen(WaitCommand(waitTime.asSeconds))
            .andThen(routine.dynamic(SysIdRoutine.Direction.kReverse))
            .andThen(WaitCommand(waitTime.asSeconds))
            .andThen(routine.quasistatic(SysIdRoutine.Direction.kForward))
            .andThen(WaitCommand(waitTime.asSeconds))
            .andThen(routine.quasistatic(SysIdRoutine.Direction.kReverse))
    }
}

class SysidMotor(val name: String, val motor: SparkMax) {
    private val m_appliedVoltage: MutVoltage = Volts.mutable(0.0)
    private val m_angle: MutAngle = Radians.mutable(0.0)
    private val m_velocity: MutAngularVelocity = RadiansPerSecond.mutable(0.0)

    fun setVoltage(voltage: Voltage?) {
        voltage ?: return
        motor.set(voltage.`in`(Volts)/ RobotController.getBatteryVoltage())
    }

    fun log(log: SysIdRoutineLog) {
        SmartDashboard.putNumber("Shooter/$name/Voltage", motor.get() * RobotController.getBatteryVoltage())
        SmartDashboard.putNumber("Shooter/$name/Position", motor.encoder.position)
        SmartDashboard.putNumber("Shooter/$name/Velocity", motor.encoder.velocity)

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
