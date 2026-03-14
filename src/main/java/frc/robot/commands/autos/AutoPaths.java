package frc.robot.commands.autos;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.ClimbingConstants;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathElement;
import frc.robot.lib.BLine.Path.PathConstraints;
import frc.robot.lib.BLine.Path.Waypoint;
import frc.robot.lib.util.ZoneUtil;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public final class AutoPaths {
    private AutoPaths() {}

    public enum AutoClimbRejectReason {
        OUTSIDE_INCLUSION_ZONE,
        TOO_CLOSE_AND_OVERROTATED
    }

    public record AutoClimbPlan(
        ClimbingConstants.AutoClimbSide side,
        Path approachPath,
        Path finalPath,
        List<Pose2d> approachWaypoints,
        Pose2d finalWaypoint,
        double estimatedDistanceMeters
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
        Logger.recordOutput("AutoPaths/AutoClimb/currentPose", currentPose);
        Logger.recordOutput("AutoPaths/AutoClimb/targetName", target.name());

        ArrayList<AutoClimbPlan> validPlans = new ArrayList<>();
        boolean hasTooCloseAndOverrotatedSide = false;
        for (ClimbingConstants.AutoClimbSide side : target.sides()) {
            boolean withinInclusionZone = ZoneUtil.isPoseInAnyZone(currentPose, side.inclusionZones(), true);
            boolean tooCloseAndOverrotated = isTooCloseAndOverrotated(
                currentPose,
                side.finalClimbPose(),
                side.preClimbPose(),
                ClimbingConstants.ROTATION_CLEARANCE_DISTANCE_METERS,
                ClimbingConstants.ROTATION_CLEARANCE_HEADING_ERROR_DEG
            );

            Logger.recordOutput("AutoPaths/AutoClimb/" + side.name() + "/preClimbPose", side.preClimbPose());
            Logger.recordOutput("AutoPaths/AutoClimb/" + side.name() + "/finalClimbPose", side.finalClimbPose());
            Logger.recordOutput("AutoPaths/AutoClimb/" + side.name() + "/withinInclusionZone", withinInclusionZone);
            Logger.recordOutput("AutoPaths/AutoClimb/" + side.name() + "/tooCloseAndOverrotated", tooCloseAndOverrotated);

            if (!withinInclusionZone) {
                continue;
            }
            if (tooCloseAndOverrotated) {
                hasTooCloseAndOverrotatedSide = true;
                continue;
            }
            validPlans.add(buildPlan(currentPose, side));
        }

        if (validPlans.isEmpty()) {
            if (hasTooCloseAndOverrotatedSide) {
                return rejected(AutoClimbRejectReason.TOO_CLOSE_AND_OVERROTATED);
            }
            return rejected(AutoClimbRejectReason.OUTSIDE_INCLUSION_ZONE);
        }

        AutoClimbPlan selectedPlan = validPlans.stream()
            .min(Comparator.comparingDouble(AutoClimbPlan::estimatedDistanceMeters))
            .orElseThrow();

        Logger.recordOutput("AutoPaths/AutoClimb/validSideCount", validPlans.size());
        Logger.recordOutput("AutoPaths/AutoClimb/selectedSideName", selectedPlan.side().name());
        Logger.recordOutput("AutoPaths/AutoClimb/selectedSideEstimatedDistanceMeters", selectedPlan.estimatedDistanceMeters());
        Logger.recordOutput("AutoPaths/AutoClimb/approachWaypointCount", selectedPlan.approachWaypoints().size());
        Logger.recordOutput("AutoPaths/AutoClimb/rejectionReason", "NONE");

        return new AutoClimbPlanningResult(selectedPlan, null);
    }

    private static AutoClimbPlan buildPlan(
        Pose2d currentPose,
        ClimbingConstants.AutoClimbSide side
    ) {
        ArrayList<Pose2d> approachWaypoints = new ArrayList<>();
        approachWaypoints.add(side.preClimbPose());

        Path approachPath = buildPathFromWaypoints(
            approachWaypoints,
            createPathConstraints(
                ClimbingConstants.APPROACH_MAX_VELOCITY_METERS_PER_SEC,
                ClimbingConstants.APPROACH_MAX_ACCELERATION_METERS_PER_SEC2,
                ClimbingConstants.APPROACH_END_TRANSLATION_TOLERANCE_METERS,
                ClimbingConstants.APPROACH_END_ROTATION_TOLERANCE_DEG
            )
        );
        Path finalPath = buildPathFromWaypoints(
            List.of(side.finalClimbPose()),
            createPathConstraints(
                ClimbingConstants.FINAL_APPROACH_MAX_VELOCITY_METERS_PER_SEC,
                ClimbingConstants.FINAL_APPROACH_MAX_ACCELERATION_METERS_PER_SEC2,
                ClimbingConstants.FINAL_APPROACH_END_TRANSLATION_TOLERANCE_METERS,
                ClimbingConstants.FINAL_APPROACH_END_ROTATION_TOLERANCE_DEG
            )
        );

        double estimatedDistanceMeters = calculateEstimatedDistanceMeters(
            currentPose,
            approachWaypoints,
            side.finalClimbPose()
        );

        Logger.recordOutput("AutoPaths/AutoClimb/" + side.name() + "/approachWaypointCount", approachWaypoints.size());
        Logger.recordOutput("AutoPaths/AutoClimb/" + side.name() + "/estimatedDistanceMeters", estimatedDistanceMeters);

        return new AutoClimbPlan(
            side,
            approachPath,
            finalPath,
            approachWaypoints,
            side.finalClimbPose(),
            estimatedDistanceMeters
        );
    }

    static boolean isTooCloseAndOverrotated(
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

    static double calculateEstimatedDistanceMeters(
        Pose2d currentPose,
        List<Pose2d> approachWaypoints,
        Pose2d finalWaypoint
    ) {
        Translation2d previousTranslation = currentPose.getTranslation();
        double distanceMeters = 0.0;

        for (Pose2d waypoint : approachWaypoints) {
            distanceMeters += previousTranslation.getDistance(waypoint.getTranslation());
            previousTranslation = waypoint.getTranslation();
        }

        distanceMeters += previousTranslation.getDistance(finalWaypoint.getTranslation());
        return distanceMeters;
    }

    private static Path buildPathFromWaypoints(List<Pose2d> waypoints, PathConstraints constraints) {
        ArrayList<PathElement> pathElements = new ArrayList<>(waypoints.size());
        for (Pose2d waypoint : waypoints) {
            pathElements.add(new Waypoint(waypoint));
        }

        return new Path(pathElements, constraints);
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
