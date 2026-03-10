package frc.robot.constants;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.lib.BLine.FlippingUtil;

public final class AlignmentConstants {
    private AlignmentConstants() {}
    
    public static final class Tower {
        private Tower() {}

        public static final double INTERMEDIARY_MAX_VELOCITY_METERS_PER_SEC = 3;
        public static final double APPROACH_MAX_VELOCITY_METERS_PER_SEC = .7;

        public static final double INTERMEDIARY_TRANSLATION_TOLERANCE_METERS = 0.08;
        public static final double INTERMEDIARY_ROTATION_TOLERANCE_DEG = 5;

        public static final class Left {
            private Left() {}

            public static final Pair<Translation2d, Translation2d> BOUNDS = new Pair<>(
                new Translation2d(1.500, 7.640),
                new Translation2d(3.456, 3.350)
            );

            public static final Pose2d INTERMEDIATE_WAYPOINT = new Pose2d(
                1.846, 
                4.636, 
                Rotation2d.fromDegrees(-90)
            ); 

            public static final Pose2d FINAL_WAYPOINT = new Pose2d(
                1.310, 
                4.636, 
                Rotation2d.fromDegrees(-90)
            );
        }
        public static final class Right {
            private Right() {}

            public static final Pair<Translation2d, Translation2d> BOUNDS = new Pair<>(
                new Translation2d(1.500, 7.640),
                new Translation2d(3.456, 0.418)
            );

            public static final Pose2d INTERMEDIATE_WAYPOINT = new Pose2d(
                5, 
                5, 
                Rotation2d.fromDegrees(0)
            );

            public static final Pose2d FINAL_WAYPOINT = new Pose2d(
                5, 
                5, 
                Rotation2d.fromDegrees(0)
            );
        }

        public static boolean isWithinBounds(Pose2d robotPose, Pair<Translation2d, Translation2d> bounds) {
            if (Constants.shouldFlipPath()) {
                robotPose = FlippingUtil.flipFieldPose(robotPose);
            }
            
            double minX = Math.min(bounds.getFirst().getX(), bounds.getSecond().getX());
            double maxX = Math.max(bounds.getFirst().getX(), bounds.getSecond().getX());
            double minY = Math.min(bounds.getFirst().getY(), bounds.getSecond().getY());
            double maxY = Math.max(bounds.getFirst().getY(), bounds.getSecond().getY());

            boolean isWithinBounds = 
                robotPose.getX() >= minX && robotPose.getX() <= maxX &&
                    robotPose.getY() >= minY && robotPose.getY() <= maxY;

            Logger.recordOutput("AlignmentConstants/isWithinBounds", isWithinBounds);
            Logger.recordOutput("AlignmentConstants/robotPose", robotPose);
            return isWithinBounds;
        }
    }
}
