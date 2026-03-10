package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.AlignmentConstants;
import frc.robot.constants.ClimbingConstants;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathElement;
import frc.robot.lib.BLine.Path.PathConstraints;
import frc.robot.lib.BLine.Path.Waypoint;
import frc.robot.lib.util.ZoneUtil;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public final class AutoPaths {
    private AutoPaths() {}

    public enum AutoClimbRejectReason {
        OUTSIDE_INCLUSION_ZONE,
        INSIDE_EXCLUSION_ZONE
    }

    public record AutoClimbPlan(
        Path approachPath,
        Path finalPath,
        List<Pose2d> approachWaypoints,
        Pose2d finalWaypoint,
        boolean usesRetreatWaypoint
    ) {
        public AutoClimbPlan {
            approachWaypoints = List.copyOf(approachWaypoints);
        }
    }

    public record AutoClimbPlanningResult(AutoClimbPlan plan, AutoClimbRejectReason rejectReason) {
        public boolean isAccepted() {
            return plan != null;
        }
    }

    public static AutoClimbPlanningResult planAutoClimb(
        Pose2d currentPose,
        ClimbingConstants.AutoClimbTarget target
    ) {
        boolean withinInclusionZone = ZoneUtil.isPoseInAnyZone(currentPose, target.inclusionZones(), true);
        boolean withinExclusionZone = ZoneUtil.isPoseInAnyZone(currentPose, target.exclusionZones(), true);

        Logger.recordOutput("AutoPaths/AutoClimb/currentPose", currentPose);
        Logger.recordOutput("AutoPaths/AutoClimb/targetPreClimbPose", target.preClimbPose());
        Logger.recordOutput("AutoPaths/AutoClimb/targetFinalPose", target.finalClimbPose());
        Logger.recordOutput("AutoPaths/AutoClimb/withinInclusionZone", withinInclusionZone);
        Logger.recordOutput("AutoPaths/AutoClimb/withinExclusionZone", withinExclusionZone);

        if (withinExclusionZone) {
            return rejected(AutoClimbRejectReason.INSIDE_EXCLUSION_ZONE);
        }
        if (!withinInclusionZone) {
            return rejected(AutoClimbRejectReason.OUTSIDE_INCLUSION_ZONE);
        }

        ArrayList<Pose2d> approachWaypoints = new ArrayList<>();
        boolean usesRetreatWaypoint = shouldInsertRetreatWaypoint(
            currentPose,
            target.finalClimbPose(),
            target.preClimbPose(),
            ClimbingConstants.ROTATION_CLEARANCE_DISTANCE_METERS,
            ClimbingConstants.ROTATION_CLEARANCE_HEADING_ERROR_DEG
        );
        if (usesRetreatWaypoint) {
            approachWaypoints.add(
                calculateRetreatPose(target, ClimbingConstants.ROTATION_CLEARANCE_RETREAT_DISTANCE_METERS)
            );
        }
        approachWaypoints.add(target.preClimbPose());

        Path approachPath = buildMirroredPathFromWaypoints(
            approachWaypoints,
            createPathConstraints(
                AlignmentConstants.Tower.INTERMEDIARY_MAX_VELOCITY_METERS_PER_SEC,
                ClimbingConstants.APPROACH_MAX_ACCELERATION_METERS_PER_SEC2,
                AlignmentConstants.Tower.INTERMEDIARY_TRANSLATION_TOLERANCE_METERS,
                AlignmentConstants.Tower.INTERMEDIARY_ROTATION_TOLERANCE_DEG
            )
        );
        Path finalPath = buildMirroredPathFromWaypoints(
            List.of(target.finalClimbPose()),
            createPathConstraints(
                AlignmentConstants.Tower.APPROACH_MAX_VELOCITY_METERS_PER_SEC,
                ClimbingConstants.FINAL_APPROACH_MAX_ACCELERATION_METERS_PER_SEC2,
                AlignmentConstants.Tower.INTERMEDIARY_TRANSLATION_TOLERANCE_METERS,
                AlignmentConstants.Tower.INTERMEDIARY_ROTATION_TOLERANCE_DEG
            )
        );

        Logger.recordOutput("AutoPaths/AutoClimb/usesRetreatWaypoint", usesRetreatWaypoint);
        Logger.recordOutput("AutoPaths/AutoClimb/approachWaypointCount", approachWaypoints.size());
        Logger.recordOutput("AutoPaths/AutoClimb/rejectionReason", "NONE");

        return new AutoClimbPlanningResult(
            new AutoClimbPlan(approachPath, finalPath, approachWaypoints, target.finalClimbPose(), usesRetreatWaypoint),
            null
        );
    }

    static boolean shouldInsertRetreatWaypoint(
        Pose2d currentPose,
        Pose2d finalClimbPose,
        Pose2d preClimbPose,
        double rotationClearanceDistanceMeters,
        double rotationClearanceHeadingErrorDeg
    ) {
        double distanceToFinalMeters = currentPose.getTranslation().getDistance(finalClimbPose.getTranslation());
        double headingErrorDeg = Math.abs(
            MathUtil.inputModulus(
                currentPose.getRotation().minus(preClimbPose.getRotation()).getDegrees(),
                -180.0,
                180.0
            )
        );
        return distanceToFinalMeters < rotationClearanceDistanceMeters
            && headingErrorDeg > rotationClearanceHeadingErrorDeg;
    }

    static Pose2d calculateRetreatPose(
        ClimbingConstants.AutoClimbTarget target,
        double retreatDistanceMeters
    ) {
        Translation2d approachVector = target.preClimbPose().getTranslation().minus(target.finalClimbPose().getTranslation());
        if (approachVector.getNorm() < 1e-6) {
            approachVector = new Translation2d(
                retreatDistanceMeters,
                0.0
            ).rotateBy(target.preClimbPose().getRotation());
        } else {
            approachVector = approachVector.div(approachVector.getNorm()).times(retreatDistanceMeters);
        }

        return new Pose2d(
            target.preClimbPose().getTranslation().plus(approachVector),
            target.preClimbPose().getRotation()
        );
    }

    private static Path buildMirroredPathFromWaypoints(List<Pose2d> waypoints, PathConstraints constraints) {
        ArrayList<PathElement> pathElements = new ArrayList<>(waypoints.size());
        for (Pose2d waypoint : waypoints) {
            pathElements.add(new Waypoint(waypoint));
        }

        Path path = new Path(pathElements, constraints);
        // SwerveDrive's shared FollowPath builder applies a global mirror to every path.
        // Pre-mirror on-the-fly climb paths so runtime execution stays in the same field frame
        // used by RobotState, zones, and the climb target constants.
        path.mirror();
        return path;
    }

    private static PathConstraints createPathConstraints(
        double maxVelocityMetersPerSec,
        double maxAccelerationMetersPerSec2,
        double endTranslationToleranceMeters,
        double endRotationToleranceDeg
    ) {
        return new PathConstraints()
            .setMaxVelocityMetersPerSec(maxVelocityMetersPerSec)
            .setMaxAccelerationMetersPerSec2(maxAccelerationMetersPerSec2)
            .setEndTranslationToleranceMeters(endTranslationToleranceMeters)
            .setEndRotationToleranceDeg(endRotationToleranceDeg);
    }

    private static AutoClimbPlanningResult rejected(AutoClimbRejectReason rejectReason) {
        Logger.recordOutput("AutoPaths/AutoClimb/rejectionReason", rejectReason.toString());
        return new AutoClimbPlanningResult(null, rejectReason);
    }
}
