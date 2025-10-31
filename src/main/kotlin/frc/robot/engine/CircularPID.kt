// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.engine

import edu.wpi.first.math.MathSharedStore
import edu.wpi.first.math.MathUsageId
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.util.sendable.SendableRegistry
import java.lang.AutoCloseable
import java.util.function.DoubleConsumer
import java.util.function.DoubleSupplier
import kotlin.math.abs

/** Implements a PID control loop.  */
class CircularPID (var p : Double, var i : Double, var d : Double) : Sendable, AutoCloseable {
    /**
     * The Proportional coefficient.
     */

    // Factor for "proportional" control

    /**
     * The Integral coefficient.
     */
    // Factor for "integral" control

    /**
     * The Differential coefficient.
     */
    // Factor for "derivative" control

    // The error range where "integral" control applies
    private var iZone = Double.Companion.POSITIVE_INFINITY
        set(iZone) {
            require(!(iZone < 0)) { "IZone must be a non-negative number!" }
            this@CircularPID.iZone = iZone
        }

    /**
     * Returns the period of this controller.
     *
     * @return the period of the controller.
     */
    // The period (in seconds) of the loop that calls the controller
    var period: Double = 0.0

    private var m_maximumIntegral = 1.0

    private var m_minimumIntegral = -1.0

    private var m_maximumInput = 0.0

    private var m_minimumInput = 0.0

    /**
     * Returns true if continuous input is enabled.
     *
     * @return True if continuous input is enabled.
     */
    // Do the endpoints wrap around? e.g. Absolute encoder
    var isContinuousInputEnabled: Boolean = false
        private set

    /**
     * Returns the difference between the setpoint and the measurement.
     *
     * @return The error.
     */
    /**
     * Returns the difference between the setpoint and the measurement.
     *
     * @return The error.
     */
    // The error at the time of the most recent call to calculate()
    var error: Double = 0.0

    /**
     * The velocity error.
     */
    var errorDerivative: Double = 0.0
        private set

    // The error at the time of the second-most-recent call to calculate() (used to compute velocity)
    private var m_prevError = 0.0

    /**
     * The accumulated error used in the integral calculation of this controller.
     */
    // The sum of the errors for use in the integral calc
    var accumulatedError: Double = 0.0
        private set

    /**
     * Returns the error tolerance of this controller. Defaults to 0.05.
     */
    // The error that is considered at setpoint.
    @get:Deprecated("Use getErrorTolerance() instead.")
    var errorTolerance: Double = 0.05
        private set

    /**
     * Returns the velocity tolerance of this controller.
     *
     * @return the velocity tolerance of the controller.
     */
    var errorDerivativeTolerance: Double = Double.Companion.POSITIVE_INFINITY
        private set
    var m_setpoint = 0.0
    var setpoint
        get() = m_setpoint
        set(setpoint) {
            m_setpoint = setpoint
            m_haveSetpoint = true

            if (this.isContinuousInputEnabled) {
                val errorBound = (m_maximumInput - m_minimumInput) / 2.0
                this.error = MathUtil.inputModulus(this@CircularPID.setpoint - m_measurement, -errorBound, errorBound)
            } else {
                this.error = m_setpoint - m_measurement
            }

            this.errorDerivative = (this.error - m_prevError) / this.period
        }
    private var m_measurement = 0.0

    private var m_haveMeasurement = false
    private var m_haveSetpoint = false

    /**
     * Allocates a PIDController with the given constants for kp, ki, and kd.
     *
     * @param kp The proportional coefficient.
     * @param ki The integral coefficient.
     * @param kd The derivative coefficient.
     * @param period The period between controller updates in seconds.
     * @throws IllegalArgumentException if kp &lt; 0
     * @throws IllegalArgumentException if ki &lt; 0
     * @throws IllegalArgumentException if kd &lt; 0
     * @throws IllegalArgumentException if period &lt;= 0
     */
    /**
     * Allocates a PIDController with the given constants for kp, ki, and kd and a default period of
     * 0.02 seconds.
     *
     * @param kp The proportional coefficient.
     * @param ki The integral coefficient.
     * @param kd The derivative coefficient.
     * @throws IllegalArgumentException if kp &lt; 0
     * @throws IllegalArgumentException if ki &lt; 0
     * @throws IllegalArgumentException if kd &lt; 0
     */
    @JvmOverloads
    fun PIDController(kp: Double, ki: Double, kd: Double, period: Double = 0.02) {
        this.p = kp
        this.i = ki
        this.d = kd

        require(!(kp < 0.0)) { "Kp must be a non-negative number!" }
        require(!(ki < 0.0)) { "Ki must be a non-negative number!" }
        require(!(kd < 0.0)) { "Kd must be a non-negative number!" }
        require(!(period <= 0.0)) { "Controller period must be a positive number!" }
        this.period = period

        instances++
        SendableRegistry.addLW(this, "PIDController", instances)

        MathSharedStore.reportUsage(MathUsageId.kController_PIDController2, instances)
    }

    override fun close() {
        SendableRegistry.remove(this)
    }

    /**
     * Sets the PID Controller gain parameters.
     *
     *
     * Set the proportional, integral, and differential coefficients.
     *
     * @param kp The proportional coefficient.
     * @param ki The integral coefficient.
     * @param kd The derivative coefficient.
     */
    fun setPID(kp: Double, ki: Double, kd: Double) {
        this.p = kp
        this.i = ki
        this.d = kd
    }


        /**
         * Sets the IZone range. When the absolute value of the position error is greater than IZone, the
         * total accumulated error will reset to zero, disabling integral gain until the absolute value of
         * the position error is less than IZone. This is used to prevent integral windup. Must be
         * non-negative. Passing a value of zero will effectively disable integral gain. Passing a value
         * of [Double.POSITIVE_INFINITY] disables IZone functionality.
         *
         * @param iZone Maximum magnitude of error to allow integral control.
         * @throws IllegalArgumentException if iZone &lt; 0
         */


        /**
         * Sets the setpoint for the PIDController.
         *
         * @param setpoint The desired setpoint.
         */


    /**
     * Returns true if the error is within the tolerance of the setpoint. The error tolerance defaults
     * to 0.05, and the error derivative tolerance defaults to ∞.
     *
     *
     * This will return false until at least one input value has been computed.
     *
     * @return Whether the error is within the acceptable bounds.
     */
    fun atSetpoint(): Boolean {
        return m_haveMeasurement
                && m_haveSetpoint
                && abs(this.error) < this.errorTolerance && abs(this.errorDerivative) < this.errorDerivativeTolerance
    }

    /**
     * Enables continuous input.
     *
     *
     * Rather then using the max and min input range as constraints, it considers them to be the
     * same point and automatically calculates the shortest route to the setpoint.
     *
     * @param minimumInput The minimum value expected from the input.
     * @param maximumInput The maximum value expected from the input.
     */
    fun enableContinuousInput(minimumInput: Double, maximumInput: Double) {
        this.isContinuousInputEnabled = true
        m_minimumInput = minimumInput
        m_maximumInput = maximumInput
    }

    /** Disables continuous input.  */
    fun disableContinuousInput() {
        this.isContinuousInputEnabled = false
    }

    /**
     * Sets the minimum and maximum contributions of the integral term.
     *
     *
     * The internal integrator is clamped so that the integral term's contribution to the output
     * stays between minimumIntegral and maximumIntegral. This prevents integral windup.
     *
     * @param minimumIntegral The minimum contribution of the integral term.
     * @param maximumIntegral The maximum contribution of the integral term.
     */
    fun setIntegratorRange(minimumIntegral: Double, maximumIntegral: Double) {
        m_minimumIntegral = minimumIntegral
        m_maximumIntegral = maximumIntegral
    }

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param errorTolerance Error which is tolerable.
     */
    fun setTolerance(errorTolerance: Double) {
        setTolerance(errorTolerance, Double.Companion.POSITIVE_INFINITY)
    }

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param errorTolerance Error which is tolerable.
     * @param errorDerivativeTolerance Error derivative which is tolerable.
     */
    fun setTolerance(errorTolerance: Double, errorDerivativeTolerance: Double) {
        this.errorTolerance = errorTolerance
        this.errorDerivativeTolerance = errorDerivativeTolerance
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @param setpoint The new setpoint of the controller.
     * @return The next controller output.
     */
    fun calculate(measurement: Double, setpoint: Double): Double {
        this@CircularPID.setpoint = setpoint
        m_haveSetpoint = true
        return calculate(measurement)
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @return The next controller output.
     */
    fun calculate(measurement: Double): Double {
        m_measurement = measurement
        m_prevError = this.error
        m_haveMeasurement = true

        /*if (this.isContinuousInputEnabled) {
            val errorBound = (m_maximumInput - m_minimumInput) / 2.0
            this.error = MathUtil.inputModulus(Rotation2d(setpoint).minus(Rotation2d(m_measurement)).radians, -errorBound, errorBound)
        } else {
            this.error = Rotation2d(setpoint).minus(Rotation2d(m_measurement)).radians
        }*/

        this.errorDerivative = (this.error - m_prevError) / this.period

        // If the absolute value of the position error is greater than IZone, reset the total error
        if (abs(this.error) > iZone) {
            this.accumulatedError = 0.0
        } else if (this.i != 0.0) {
            this.accumulatedError =
                MathUtil.clamp(
                    this.accumulatedError + this.error * this.period,
                    m_minimumIntegral / this.i,
                    m_maximumIntegral / this.i
                )
        }

        return this.p * this.error + this.i * this.accumulatedError + this.d * this.errorDerivative
    }

    /** Resets the previous error and the integral term.  */
    fun reset() {
        this.error = 0.0
        m_prevError = 0.0
        this.accumulatedError = 0.0
        this.errorDerivative = 0.0
        m_haveMeasurement = false
    }

    override fun initSendable(builder: SendableBuilder) {
        builder.setSmartDashboardType("PIDController")
        builder.addDoubleProperty("p", DoubleSupplier { this.p }, DoubleConsumer { kp: Double -> this.p = kp })
        builder.addDoubleProperty("i", DoubleSupplier { this.i }, DoubleConsumer { ki: Double -> this.i = ki })
        builder.addDoubleProperty("d", DoubleSupplier { this.d }, DoubleConsumer { kd: Double -> this.d = kd })
        builder.addDoubleProperty(
            "izone",
            DoubleSupplier { this.iZone },
            DoubleConsumer { toSet: Double ->
                try {
                    this.iZone = toSet
                } catch (e: IllegalArgumentException) {
                    MathSharedStore.reportError("IZone must be a non-negative number!", e.getStackTrace())
                }
            })
        builder.addDoubleProperty("setpoint", DoubleSupplier { this.setpoint }, DoubleConsumer { setpoint: Double ->
            this.setpoint = setpoint
        })
        builder.addDoubleProperty("measurement", DoubleSupplier { m_measurement }, null)
        builder.addDoubleProperty("error", DoubleSupplier { this.error }, null)
        builder.addDoubleProperty("error derivative", DoubleSupplier { this.errorDerivative }, null)
        builder.addDoubleProperty("previous error", DoubleSupplier { this.m_prevError }, null)
        builder.addDoubleProperty("total error", DoubleSupplier { this.accumulatedError }, null)
    }

    companion object {
        private var instances = 0
    }
}