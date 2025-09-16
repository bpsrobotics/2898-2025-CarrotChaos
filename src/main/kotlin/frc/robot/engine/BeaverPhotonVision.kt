package frc.robot.engine

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.photonvision.targeting.PhotonPipelineResult

class BeaverPhotonVision(vararg val cameras: VisionCamera) : SubsystemBase() {
    val listeners = BiSignal<PhotonPipelineResult, VisionCamera>()

    override fun periodic() {
        for(camera in cameras){
            for (result in camera.results) {
                listeners.update(result, camera)
            }
        }
    }
    fun getStandardDev(rotationSTD: Double): Matrix<N3, N1>{
        val stdv = Matrix(Nat.N3(), Nat.N1())
        stdv.set(0,0, 3.0)
        stdv.set(1,0, 3.0)
        stdv.set(2,0, rotationSTD)
        return stdv
    }
}