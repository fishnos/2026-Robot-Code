package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.ClimbingConstants;
import frc.robot.constants.ZoneConstants.RectangleZone;
import org.junit.jupiter.api.Test;

class AutoPathsTest {
    private static final RectangleZone TEST_INCLUSION_ZONE = new RectangleZone(
        "test_inclusion",
        new Translation2d(0.0, 0.0),
        new Translation2d(5.0, 5.0)
    );

    private static final Pose2d TEST_PRE_CLIMB_POSE = new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(0.0));
    private static final Pose2d TEST_FINAL_CLIMB_POSE = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(0.0));

    private static ClimbingConstants.AutoClimbTarget createTarget(RectangleZone... exclusionZones) {
        return new ClimbingConstants.AutoClimbTarget(
            "test_target",
            TEST_PRE_CLIMB_POSE,
            TEST_FINAL_CLIMB_POSE,
            java.util.List.of(TEST_INCLUSION_ZONE),
            java.util.List.of(exclusionZones)
        );
    }

    @Test
    void planAutoClimb_rejectsWhenRobotStartsOutsideInclusionZone() {
        AutoPaths.AutoClimbPlanningResult result = AutoPaths.planAutoClimb(
            new Pose2d(6.0, 2.0, Rotation2d.fromDegrees(0.0)),
            createTarget()
        );

        assertFalse(result.isAccepted());
        assertEquals(AutoPaths.AutoClimbRejectReason.OUTSIDE_INCLUSION_ZONE, result.rejectReason());
    }

    @Test
    void planAutoClimb_rejectsWhenRobotStartsInsideExclusionZone() {
        RectangleZone blocker = new RectangleZone(
            "blocker",
            new Translation2d(3.5, 1.0),
            new Translation2d(4.5, 3.0)
        );

        AutoPaths.AutoClimbPlanningResult result = AutoPaths.planAutoClimb(
            new Pose2d(4.0, 2.0, Rotation2d.fromDegrees(0.0)),
            createTarget(blocker)
        );

        assertFalse(result.isAccepted());
        assertEquals(AutoPaths.AutoClimbRejectReason.INSIDE_EXCLUSION_ZONE, result.rejectReason());
    }

    @Test
    void planAutoClimb_usesOnlyThePreClimbWaypointWhenRetreatIsNotNeeded() {
        AutoPaths.AutoClimbPlanningResult result = AutoPaths.planAutoClimb(
            new Pose2d(4.0, 2.0, Rotation2d.fromDegrees(0.0)),
            createTarget()
        );

        assertTrue(result.isAccepted());
        assertNotNull(result.plan());
        assertFalse(result.plan().usesRetreatWaypoint());
        assertEquals(1, result.plan().approachWaypoints().size());
        assertEquals(TEST_PRE_CLIMB_POSE, result.plan().approachWaypoints().get(0));
        assertEquals(TEST_FINAL_CLIMB_POSE, result.plan().finalWaypoint());
        assertTrue(result.plan().approachPath().isValid());
        assertTrue(result.plan().finalPath().isValid());
    }

    @Test
    void planAutoClimb_insertsRetreatWaypointWhenRobotIsTooCloseAndMisaligned() {
        AutoPaths.AutoClimbPlanningResult result = AutoPaths.planAutoClimb(
            new Pose2d(1.25, 2.0, Rotation2d.fromDegrees(90.0)),
            createTarget()
        );

        assertTrue(result.isAccepted());
        assertNotNull(result.plan());
        assertTrue(result.plan().usesRetreatWaypoint());
        assertEquals(2, result.plan().approachWaypoints().size());
        assertTrue(
            result.plan().approachWaypoints().get(0).getX() > TEST_PRE_CLIMB_POSE.getX(),
            "Retreat waypoint should move farther back along the climb lane before re-approach"
        );
        assertEquals(TEST_PRE_CLIMB_POSE, result.plan().approachWaypoints().get(1));
    }
}
