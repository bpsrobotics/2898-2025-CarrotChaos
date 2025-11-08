package frc.robot.subsystems

import beaverlib.odometry.BeaverPhotonVision
import edu.wpi.first.apriltag.AprilTag
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import org.photonvision.PhotonPoseEstimator

val customField : MutableList<AprilTag> = mutableListOf(
    AprilTag(1, Pose3d())
)

//val customVision = AprilTagFieldLayout.(mutableListOf<AprilTag>(
//    AprilTag(4, Pose3d(0.0, 0.0, 0.0, Rotation3d()
//    ))))

val Vision = BeaverPhotonVision(
    /*VisionCamera(
        //"Arducam_John",
        "HD_USB_Camera",
        Transform3d(0.18,-0.33,0.2, Rotation3d()),
        layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded),
        strategy = PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        fallbackStrategy = PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
    )*/
)