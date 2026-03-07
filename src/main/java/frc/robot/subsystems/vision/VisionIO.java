package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean connected = false;
        public PoseObservation[] robotPoseObservations = new PoseObservation[0];

        public int[] tagIds = new int[0];
        public Pose3d[] tagPoses = new Pose3d[0];
        public int rawMegatag1ObservationCount = 0;
        public int rawMegatag2ObservationCount = 0;
        public int rawObservationCount = 0;
        public int coalescedObservationCount = 0;
        public int coalescedDropCount = 0;
        public int[] coalescedGroupSizes = new int[0];
        public String[] coalescedTranslationSourceTypes = new String[0];
        public String[] coalescedTranslationDecisionReasons = new String[0];
        public String[] coalescedRotationSourceTypes = new String[0];
        public String[] coalescedRotationDecisionReasons = new String[0];
        public TargetObservation latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
    }

    /** Represents the angle to a simple target, not used for pose estimation. */
    public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

    /** Represents a robot pose sample used for pose estimation. */
    public static record PoseObservation(
        double timestamp,
        Pose3d pose,
        double ambiguity,
        int tagCount,
        double averageTagDistance,
        PoseObservationType type,
        double rotationAmbiguity,
        int rotationTagCount,
        double rotationAverageTagDistance,
        PoseObservationType rotationType) {

        public PoseObservation(
            double timestamp,
            Pose3d pose,
            double ambiguity,
            int tagCount,
            double averageTagDistance,
            PoseObservationType type
        ) {
            this(
                timestamp,
                pose,
                ambiguity,
                tagCount,
                averageTagDistance,
                type,
                ambiguity,
                tagCount,
                averageTagDistance,
                type
            );
        }
    }

    public static enum PoseObservationType {
        MEGATAG_1,
        MEGATAG_2,
        PHOTONVISION
    }

    public default boolean publishRobotOrientation() {
        return false;
    }

    public default void updateInputs(VisionIOInputs inputs) {}
}
