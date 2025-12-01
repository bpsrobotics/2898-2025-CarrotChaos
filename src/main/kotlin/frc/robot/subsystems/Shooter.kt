package frc.robot.subsystems

import beaverlib.controls.PIDConstants
import beaverlib.controls.SimpleMotorFeedForwardConstants
import beaverlib.utils.Units.Angular.AngularVelocity
import beaverlib.utils.Units.Angular.RPM
import beaverlib.utils.Units.Angular.asRPM
import beaverlib.utils.Units.Linear.inches
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import frc.robot.commands.shooter.StopShooter
import frc.robot.engine.PIDFF
import frc.robot.subsystems.Shooter.setSpeeds

object Shooter : SubsystemBase() {
    private val motorTop = SparkMax(RobotMap.ShooterTopId, SparkLowLevel.MotorType.kBrushless)
    private val motorBottom = SparkMax(RobotMap.ShooterBotId, SparkLowLevel.MotorType.kBrushless)
    private val gateMotor = SparkMax(RobotMap.FeederId, SparkLowLevel.MotorType.kBrushless)
    private val mechanism2d: Mechanism2d = Mechanism2d(3.0, 3.0)

    private val shooterConfig: SparkMaxConfig = SparkMaxConfig()
    private val gateConfig: SparkMaxConfig = SparkMaxConfig()

    object Constants {
        val WheelRadius = 6.inches // todo
        val pidConstants = PIDConstants(0.1, 0.0, 0.0)
        val ffConstants = SimpleMotorFeedForwardConstants(0.0, 0.0, 0.0)
    }

    val topMotorPIDFF: PIDFF = PIDFF(Constants.pidConstants, Constants.ffConstants)
    val bottomMotorPIDFF: PIDFF = PIDFF(Constants.pidConstants, Constants.ffConstants)
    val gateSpeed
        get() = gateMotor.encoder.velocity.RPM

    val topMotorSpeed
        get() = motorTop.encoder.velocity.RPM

    val bottomMotorSpeed
        get() = motorBottom.encoder.velocity.RPM

    val shooterSpeed
        get() = (topMotorSpeed + bottomMotorSpeed) / 2.0

    // val Carrot1 = MechanismLigament2d("Carrot1", 2.5, 0.0, 20.0, Color8Bit(255, 172, 28))

    init {
        /*mechanism2d.getRoot("Carrot1Pos", 1.0, 1.0).append<MechanismLigament2d>(Carrot1)
        mechanism2d
            .getRoot("Base", 1.0, 0.4)
            .append<MechanismLigament2d>(
                MechanismLigament2d("Base", 10.0, 0.0, 20.0, Color8Bit(0, 0, 0))
            )*/

        // Intake motor initialisation stuff
        shooterConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(20)

        motorTop.configure(
            shooterConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters,
        )
        motorBottom.configure(
            shooterConfig.inverted(true),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters,
        )

        // Intake motor initialisation stuff
        gateConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(20)
        gateMotor.configure(
            gateConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters,
        )

        defaultCommand = StopShooter()
    }

    /** Runs both shooter motors using openloop at the given [percent] */
    fun runAtPercent(percent: Double) {
        motorTop.set(percent)
        motorBottom.set(percent)
    }

    /** Runs the gate motor using openloop at the given [percent] */
    fun runGateAtPercent(percent: Double) {
        gateMotor.set(percent)
    }

    /** Stops all motors (top, bottom, and gate) from running */
    fun stop() {
        motorTop.stopMotor()
        motorBottom.stopMotor()
        gateMotor.stopMotor()
    }

    /** Sets both shooter motors to stop running */
    fun stopShooter() {
        motorTop.stopMotor()
        motorBottom.stopMotor()
    }

    /** Sets the gate motor to stop running */
    fun stopGate() {
        gateMotor.stopMotor()
    }

    /** Sets the PIDFF setpoint for both motors to [speed] */
    fun setSpeed(speed: AngularVelocity) {
        setSpeeds(speed, speed)
    }

    /** Sets the PIDFF setpoint for each motor to [topSpeed] and [bottomSpeed] */
    fun setSpeeds(bottomSpeed: AngularVelocity, topSpeed: AngularVelocity) {
        topMotorPIDFF.setpoint = topSpeed.asRPM
        bottomMotorPIDFF.setpoint = bottomSpeed.asRPM
    }

    /** Runs the shooter using the current setpoint (given by [setSpeeds]) */
    fun runPIDFF() {
        motorTop.setVoltage(topMotorPIDFF.calculate(motorTop.encoder.velocity))
        motorBottom.setVoltage(bottomMotorPIDFF.calculate(motorBottom.encoder.velocity))
    }

    /** Returns true if both PIDs [PIDFF.atSetpoint] returns true. */
    fun isAtSpeed(): Boolean {
        return topMotorPIDFF.atSetpoint() && bottomMotorPIDFF.atSetpoint()
    }

    /** Put the top and bottom motor encoder RPMS to [SmartDashboard] */
    override fun periodic() {
        SmartDashboard.putNumber("Shooter/TopMotorRPM", motorTop.encoder.velocity)
        SmartDashboard.putNumber("Shooter/BottomMotorRPM", motorBottom.encoder.velocity)
        /*mechanism2d
            .getRoot("Carrot1Pos", 0.0, 0.0)
            .setPosition(5 + 4 * kotlin.math.sin(Timer.getFPGATimestamp() / 2), 1.0)
        SmartDashboard.putData("Mech2D", mechanism2d)*/
    }
}
