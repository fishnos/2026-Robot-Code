package frc.robot.constants.vision;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Optional;
import java.util.logging.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.constants.Constants;

public class VisionConstants {
  // AprilTag layout
  private static AprilTagFieldLayout aprilTagLayout;
  private static final AprilTagFieldLayout EMPTY_APRIL_TAG_LAYOUT =
      new AprilTagFieldLayout(List.of(), 0.0, 0.0);
  private static final Logger logger = Logger.getLogger(VisionConstants.class.getName());
  private static final String[] DEPLOY_LAYOUT_CANDIDATES = {
      "vision/2026-rebuilt-welded.json",
      "vision/2026-rebuilt-andymark.json",
      "custom_apriltag_field_layout.json",
      "custom_apriltag_field_layout.fmap"
  };

  private static AprilTagFieldLayout loadDeployLayoutIfPresent() throws IOException {
    File deployDirectory = Filesystem.getDeployDirectory();
    for (String relativePath : DEPLOY_LAYOUT_CANDIDATES) {
      File layoutFile = new File(deployDirectory, relativePath);
      if (layoutFile.exists() && layoutFile.isFile()) {
        logger.info("Loaded AprilTag field layout from deploy: " + layoutFile.getAbsolutePath());
        return new AprilTagFieldLayout(layoutFile.getAbsolutePath());
      }
    }
    throw new IOException("No deploy AprilTag field layout found in known locations.");
  }

  private static AprilTagFieldLayout loadWpilib2026LayoutIfPresent() {
    // Use reflection so this still compiles with older WPILib releases.
    String[] fieldCandidates = {"k2026RebuiltWelded", "k2026RebuiltAndymark"};
    for (String candidate : fieldCandidates) {
      try {
        AprilTagFields field = (AprilTagFields) AprilTagFields.class.getField(candidate).get(null);
        logger.info("Loaded WPILib AprilTag field layout: " + candidate);
        return AprilTagFieldLayout.loadField(field);
      } catch (ReflectiveOperationException ignored) {
        // Try next candidate.
      }
    }
    return null;
  }

  private static AprilTagFieldLayout loadAprilTagLayout() {
    try {
      return loadDeployLayoutIfPresent();
    } catch (IOException deployLoadFailure) {
      AprilTagFieldLayout wpilib2026Layout = loadWpilib2026LayoutIfPresent();
      if (wpilib2026Layout != null) {
        return wpilib2026Layout;
      }

      logger.warning(
          "Falling back to WPILib default AprilTag layout ("
              + AprilTagFields.kDefaultField
              + "). Deploy load failure: "
              + deployLoadFailure.getMessage());
      return AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    }
  }

  public static synchronized Optional<AprilTagFieldLayout> getAprilTagLayout() {
    if (!Constants.VERBOSE_LOGGING_ENABLED) {
      return Optional.empty();
    }

    if (aprilTagLayout == null) {
      aprilTagLayout = loadAprilTagLayout();
    }

    return Optional.of(aprilTagLayout);
  }

  public static AprilTagFieldLayout getAprilTagLayoutOrEmpty() {
    return getAprilTagLayout().orElse(EMPTY_APRIL_TAG_LAYOUT);
  }
}
