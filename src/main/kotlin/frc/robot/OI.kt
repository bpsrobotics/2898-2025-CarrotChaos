package frc.robot

import beaverlib.utils.geometry.Vector2

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.Constants.ButtonConstants.ALGAE_B1
import frc.robot.Constants.ButtonConstants.ALGAE_B2
import frc.robot.Constants.ButtonConstants.AUTO_INTAKE
import frc.robot.Constants.ButtonConstants.BASE_STAGE
import frc.robot.Constants.ButtonConstants.CORAL_L2
import frc.robot.Constants.ButtonConstants.CORAL_L3
import frc.robot.Constants.ButtonConstants.CORAL_L4
import frc.robot.Constants.ButtonConstants.ELEV_BW
import frc.robot.Constants.ButtonConstants.ELEV_FW
import frc.robot.Constants.ButtonConstants.PIVOT_BW
import frc.robot.Constants.ButtonConstants.PIVOT_FW
import frc.robot.Constants.ButtonConstants.TOGGLE_STATE
import frc.robot.commands.AlignAprilTag
import frc.robot.commands.AlignOdometry
import frc.robot.commands.FollowApriltag
import frc.robot.commands.FollowApriltagGood
import frc.robot.commands.OI.Rumble
import frc.robot.commands.ScoreTag
import frc.robot.commands.elevator.MoveElevator
import frc.robot.commands.elevator.MoveElevatorBy
import frc.robot.commands.intake.RunIntake
import frc.robot.commands.intake.RunOuttake
import frc.robot.commands.OI.NavXReset
import frc.robot.commands.elevator.StabilizeElevator
import frc.robot.commands.wrist.MoveWrist
import frc.robot.commands.wrist.MoveWristBy
import frc.robot.commands.wrist.StabilizeWrist
import frc.robot.commands.wrist.ToggleState

import kotlin.math.pow
import kotlin.math.sign

/**
 * The Operating Interface object.
 * This is where you put joystick, button, or keyboard inputs.
 *
 * A note about delegated properties, which are used in this object:
 *  A delegated property is where getting (or setting) a field is outsourced
 *  to another object.  Then, whenever you read the property, it asks the
 *  object the property is delegated to for the value.
 */
@Suppress("unused")
object OI : SubsystemBase() {
    init {
        defaultCommand = Rumble(GenericHID.RumbleType.kBothRumble, 0.0)
    }

    val navXResetCommand: NavXReset = NavXReset()
    val followTagCommand = FollowApriltagGood(18)
        //AlignOdometry(18,Pose2d(2.7, 4.27, Rotation2d(0.0)))
        /*SequentialCommandGroup(
            ParallelCommandGroup(
                AlignOdometry(18, Pose2d(2.5, 4.29, Rotation2d(0.0))),
                SequentialCommandGroup(MoveWrist(Constants.PivotConstants.PivotState.VerticalBranch.position),
                    ParallelRaceGroup( MoveElevator(Constants.ElevatorConstants.ElevatorState.L4.position),
                    StabilizeWrist()))
            ),*
        ParallelRaceGroup(
            StabilizeWrist(),
            StabilizeElevator(),
            AlignOdometry(18, Pose2d(3.11, 4.29, Rotation2d(0.0)))
        )

    )*/


    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * [Trigger.Trigger] constructor with an arbitrary
     * predicate, or via the named factories in [ ]'s subclasses for [ ]/[ PS4][edu.wpi.first.wpilibj2.command.button.CommandPS4Controller] controllers or [Flight][CommandJoystick].
     */
    fun configureBindings() {
        resetGyro.whileTrue(navXResetCommand)
        followTag.whileTrue(followTagCommand)


        toggleWrist.onTrue(ToggleState())

        autoIntake.onTrue(RunIntake())

        highHatForward.whileTrue(RunOuttake(0.5))
        highHatBack.whileTrue(RunOuttake(-1.0))

        elevFWStepper.whileTrue(MoveElevatorBy(0.005))
        elevBWStepper.whileTrue(MoveElevatorBy(-0.005))

//        pivotFWStepper.whileTrue(VoltageWrist(0.2))
        pivotFWStepper.whileTrue(MoveWristBy(-0.03))
        pivotBWStepper.whileTrue(MoveWristBy(0.03))

//        pivotBWStepper.whileTrue(VoltageWrist(-0.2))


        OI.moveA1.onTrue(
            SequentialCommandGroup(
                MoveElevator(Constants.ElevatorConstants.ElevatorState.A1.position),
                MoveWrist(Constants.PivotConstants.PivotState.Algae.position)
            ))
        OI.moveA2.onTrue(      SequentialCommandGroup(
            MoveElevator(Constants.ElevatorConstants.ElevatorState.A2.position),
            MoveWrist(Constants.PivotConstants.PivotState.Algae.position)

        ))
        OI.moveL1.onTrue(
            MoveElevator(Constants.ElevatorConstants.ElevatorState.Stow.position)
        )
        OI.moveL2.onTrue(  SequentialCommandGroup(MoveWrist(Constants.PivotConstants.PivotState.Traverse.position),
            MoveElevator(Constants.ElevatorConstants.ElevatorState.L2.position),
            MoveWrist(Constants.PivotConstants.PivotState.AngleBranch.position)

        ))
        OI.moveL3.onTrue(        SequentialCommandGroup(MoveWrist(Constants.PivotConstants.PivotState.AngleBranch.position),
            MoveElevator(Constants.ElevatorConstants.ElevatorState.L3.position),
        ))
        OI.moveL4.onTrue(SequentialCommandGroup(MoveWrist(Constants.PivotConstants.PivotState.VerticalBranch.position),

            MoveElevator(Constants.ElevatorConstants.ElevatorState.L4.position)))
    }

    /**
     * Threshold below which [process] will return 0.
     * 0.1 historically used, but optimal value unknown.
     */
    private const val DEADZONE_THRESHOLD = 0.1

    /**
     * Utility function for controller axis, optional deadzone and square/cube for extra fine-grain control
     */
    private fun process(
        input: Double,
        deadzone: Boolean = false,
        square: Boolean = false,
        cube: Boolean = false
    ): Double {
        var output = 0.0

        if (deadzone) {
            output = MathUtil.applyDeadband(input, DEADZONE_THRESHOLD)
        }

        if (square) {
            // To keep the signage for output, we multiply by sign(output). This keeps negative inputs resulting in negative outputs.
            output = output.pow(2) * sign(output)
        }

        if (cube) {
            // Because cubing is an odd number of multiplications, we don't need to multiply by sign(output) here.
            output = output.pow(3)
        }

        return output
    }

    // conflicts with the other definition, name it something else after compilation
    @JvmName("process1")
    fun Double.process(deadzone: Boolean = false, square: Boolean = false, cube: Boolean = false) =
        process(this, deadzone, square, cube)

    val driverController = CommandXboxController(0)
    private val operatorController = CommandJoystick(1)

    // Right joystick y-axis.  Controller mapping can be tricky, the best way is to use the driver station to see what buttons and axis are being pressed.
    // Squared for better control on turn, cubed on throttle
    /** Driver controller's throttle on the left joystick for the X Axis, from -1 (left) to 1 (right) */
    val translationX
        get() = process(driverController.leftX, deadzone = true, square = false)

    /** Driver controller's throttle on the left joystick for the Y Axis, from -1 (down) to 1 (up) */
    val translationY
        get() = process(driverController.leftY, deadzone = true, square = false)

    /** Driver controller's throttle on the right joystick for the X Axis, from -1 (left) to 1 (right) */
    val turnX
        get() = process(driverController.rightX, deadzone = true, square = false)
    /** Driver controller's throttle on the right joystick for the Y Axis, from -1 (down) to 1 (up) */
    val turnY
        get() = process(driverController.rightY, deadzone = true, square = false)
    val leftTrigger
        get() = driverController.leftTriggerAxis
    val rightTrigger
        get() = driverController.rightTriggerAxis

    val resetGyro: Trigger = driverController.rightBumper()
    val followTag: Trigger = driverController.leftBumper()
    val sysidFQ: Trigger = driverController.x()
    val sysidBQ: Trigger = driverController.y()
    val sysidFD: Trigger = driverController.b()
    val sysidBD: Trigger = driverController.a()

    val highHatForward: Trigger = operatorController.pov(0)
    val highHatBack: Trigger = operatorController.pov(180)

    val moveL1 = operatorController.button(BASE_STAGE)
    val moveL2 = operatorController.button(CORAL_L2)
    val moveL3 = operatorController.button(CORAL_L3)
    val moveL4 = operatorController.button(CORAL_L4)
    val moveA1 = operatorController.button(ALGAE_B1)
    val moveA2 = operatorController.button(ALGAE_B2)

    val coralAlignLeft = driverController.povLeft()
    val coralAlignRight = driverController.povRight()


    val autoIntake = operatorController.button(AUTO_INTAKE)
    val toggleWrist = operatorController.button(TOGGLE_STATE)

    val pivotFWStepper = operatorController.button(PIVOT_FW)
    val pivotBWStepper = operatorController.button(PIVOT_BW)

    val elevFWStepper = operatorController.button(ELEV_FW)
    val elevBWStepper = operatorController.button(ELEV_BW)
//    val hatVector get() = when (operatorController.pov) {
//        0 -> Vector2(0.0,1.0)
//        90 -> Vector2(1.0,0.0)
//        180 -> Vector2(0.0,-1.0)
//        270 -> Vector2(-1.0,0.0)
//        else -> Vector2.zero()
//    }

    val intakeSpeed get() = operatorController.throttle

    enum class Direction {
        LEFT, RIGHT, UP, DOWN, UPLEFT, UPRIGHT, DOWNLEFT, DOWNRIGHT, INACTIVE;

        fun mirrored() = when (this) {
            LEFT  -> RIGHT
            RIGHT -> LEFT
            else  -> this
        }
        fun toVector() = when(this) {
            LEFT -> Vector2(-1.0,0.0)
            RIGHT -> Vector2(1.0,0.0)
            UP -> Vector2(0.0,1.0)
            DOWN -> Vector2(0.0,-1.0)
            INACTIVE -> Vector2.zero()
            UPLEFT -> Vector2(-1.0,1.0)
            UPRIGHT -> Vector2(1.0, 1.0)
            DOWNLEFT -> Vector2(-1.0, -1.0)
            DOWNRIGHT -> Vector2(1.0, -1.0)
        }
    }
}