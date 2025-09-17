package frc.robot.subsystems

import edu.wpi.first.apriltag.AprilTag
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Transform3d
import frc.robot.engine.BeaverPhotonVision
import frc.robot.engine.VisionCamera
import org.photonvision.PhotonPoseEstimator

val customField : MutableList<AprilTag> = mutableListOf(
    AprilTag(1, Pose3d())
)

val Vision = BeaverPhotonVision(
    VisionCamera(
        "Arducam_OV9281_USB_Camera",
        Transform3d(),
        layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded),
        strategy = PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        fallbackStrategy = PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
    )
)