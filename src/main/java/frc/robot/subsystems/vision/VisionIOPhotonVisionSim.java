package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import frc.robot.constants.vision.VisionConstants;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  private static VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIOPhotonVisionSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    super(name, robotToCamera);
    this.poseSupplier = poseSupplier;

    // Initialize vision sim
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      var aprilTagLayout = VisionConstants.getAprilTagLayout();
      if (aprilTagLayout.isPresent()) {
        visionSim.addAprilTags(aprilTagLayout.get());
      }
    }

    // Add sim camera
    var cameraProperties = new SimCameraProperties();
    cameraSim = new PhotonCameraSim(camera, cameraProperties, VisionConstants.getAprilTagLayoutOrEmpty());
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}
