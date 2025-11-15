package frc.robot.commands.tunnel

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Tunnel

class RunTunnelForTime(val percent: Double, val time: Double = -1.0) : Command() {
    val isInfinite = time < 0
    val timer = Timer()

    init {
        addRequirements(Tunnel)
    }

    override fun initialize() {
        if (!isInfinite) timer.restart()
        Tunnel.runAtPercent(percent)
    }

    override fun isFinished(): Boolean {
        return !isInfinite && timer.hasElapsed(time)
    }

    override fun end(interrupted: Boolean) {
        Tunnel.runAtPercent(0.0)
    }
}
