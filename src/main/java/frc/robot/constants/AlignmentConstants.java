package frc.robot.constants;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.ZoneConstants.RectangleZone;
import frc.robot.lib.util.ZoneUtil;

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
                ZoneConstants.Tower.LEFT.cornerA(),
                ZoneConstants.Tower.LEFT.cornerB()
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
                ZoneConstants.Tower.RIGHT.cornerA(),
                ZoneConstants.Tower.RIGHT.cornerB()
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
            RectangleZone zone = new RectangleZone(
                "alignment_bounds",
                bounds.getFirst(),
                bounds.getSecond()
            );
            boolean isWithinBounds = ZoneUtil.isPoseInZone(robotPose, zone, true);

            Logger.recordOutput("AlignmentConstants/isWithinBounds", isWithinBounds);
            if (Constants.VERBOSE_LOGGING_ENABLED) {
                Logger.recordOutput("AlignmentConstants/robotPose", robotPose);
            }
            return isWithinBounds;
        }
    }
}
