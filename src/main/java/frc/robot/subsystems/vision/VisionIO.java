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
        public TargetObservation latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
        public BallObservation[] ballObservations = new BallObservation[0];
    }

    /** Represents the angle to a simple target, not used for pose estimation. */
    public static record TargetObservation(Rotation2d tx, Rotation2d ty) {
    }

    /** Represents a detected ball from the neural detector. */
    public static record BallObservation(double tx, double ty, double area, double confidence) {
    }

    /** Represents a robot pose sample used for pose estimation. */
    public static record PoseObservation(
            double timestamp,
            Pose3d pose,
            double ambiguity,
            int tagCount,
            double averageTagDistance,
            PoseObservationType type) {
    }

    public static enum PoseObservationType {
        MEGATAG_1,
        MEGATAG_2,
        PHOTONVISION
    }

    public default void updateInputs(VisionIOInputs inputs) {
    }
}