package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.autos.AutoPaths;
import frc.robot.constants.ClimbingConstants;
import frc.robot.constants.ZoneConstants.RectangleZone;
import java.util.List;
import org.junit.jupiter.api.Test;

class AutoPathsTest {
    private static final RectangleZone TOP_INCLUSION_ZONE = new RectangleZone(
        "top_inclusion",
        new Translation2d(0.0, 2.5),
        new Translation2d(5.0, 5.0)
    );

    private static final RectangleZone BOTTOM_INCLUSION_ZONE = new RectangleZone(
        "bottom_inclusion",
        new Translation2d(0.0, 0.0),
        new Translation2d(5.0, 2.5)
    );

    private static final RectangleZone SHARED_INCLUSION_ZONE = new RectangleZone(
        "shared_inclusion",
        new Translation2d(0.0, 0.0),
        new Translation2d(5.0, 5.0)
    );

    private static final Pose2d TOP_PRE_CLIMB_POSE = new Pose2d(2.0, 4.0, Rotation2d.fromDegrees(0.0));
    private static final Pose2d TOP_FINAL_CLIMB_POSE = new Pose2d(1.0, 4.0, Rotation2d.fromDegrees(0.0));
    private static final Pose2d BOTTOM_PRE_CLIMB_POSE = new Pose2d(2.0, 1.0, Rotation2d.fromDegrees(180.0));
    private static final Pose2d BOTTOM_FINAL_CLIMB_POSE = new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(180.0));

    private static ClimbingConstants.AutoClimbTarget createTarget(
        RectangleZone topInclusionZone,
        RectangleZone bottomInclusionZone
    ) {
        return new ClimbingConstants.AutoClimbTarget(
            "test_target",
            List.of(
                new ClimbingConstants.AutoClimbSide(
                    "top_side",
                    TOP_PRE_CLIMB_POSE,
                    TOP_FINAL_CLIMB_POSE,
                    List.of(topInclusionZone)
                ),
                new ClimbingConstants.AutoClimbSide(
                    "bottom_side",
                    BOTTOM_PRE_CLIMB_POSE,
                    BOTTOM_FINAL_CLIMB_POSE,
                    List.of(bottomInclusionZone)
                )
            )
        );
    }

    @Test
    void planAutoClimb_rejectsWhenRobotStartsOutsideEveryInclusionZone() {
        AutoPaths.AutoClimbPlanningResult result = AutoPaths.planAutoClimb(
            new Pose2d(6.0, 2.0, Rotation2d.fromDegrees(0.0)),
            createTarget(TOP_INCLUSION_ZONE, BOTTOM_INCLUSION_ZONE)
        );

        assertFalse(result.isAccepted());
        assertEquals(AutoPaths.AutoClimbRejectReason.OUTSIDE_INCLUSION_ZONE, result.rejectReason());
    }

    @Test
    void planAutoClimb_selectsTheOnlyValidSide() {
        AutoPaths.AutoClimbPlanningResult result = AutoPaths.planAutoClimb(
            new Pose2d(4.0, 4.0, Rotation2d.fromDegrees(0.0)),
            createTarget(TOP_INCLUSION_ZONE, BOTTOM_INCLUSION_ZONE)
        );

        assertTrue(result.isAccepted());
        assertNotNull(result.plan());
        assertEquals("top_side", result.plan().side().name());
        assertEquals(TOP_FINAL_CLIMB_POSE, result.plan().finalWaypoint());
        assertTrue(result.plan().approachPath().isValid());
        assertTrue(result.plan().finalPath().isValid());
    }

    @Test
    void planAutoClimb_selectsTheClosestValidSideWhenBothSidesAreAvailable() {
        AutoPaths.AutoClimbPlanningResult result = AutoPaths.planAutoClimb(
            new Pose2d(3.0, 1.2, Rotation2d.fromDegrees(180.0)),
            createTarget(SHARED_INCLUSION_ZONE, SHARED_INCLUSION_ZONE)
        );

        assertTrue(result.isAccepted());
        assertNotNull(result.plan());
        assertEquals("bottom_side", result.plan().side().name());
        assertEquals(BOTTOM_FINAL_CLIMB_POSE, result.plan().finalWaypoint());
    }

    @Test
    void planAutoClimb_usesOnlyThePreClimbWaypointWhenRetreatIsNotNeeded() {
        AutoPaths.AutoClimbPlanningResult result = AutoPaths.planAutoClimb(
            new Pose2d(4.0, 4.0, Rotation2d.fromDegrees(0.0)),
            createTarget(TOP_INCLUSION_ZONE, BOTTOM_INCLUSION_ZONE)
        );

        assertTrue(result.isAccepted());
        assertNotNull(result.plan());
        assertEquals(1, result.plan().approachWaypoints().size());
        assertEquals(TOP_PRE_CLIMB_POSE, result.plan().approachWaypoints().get(0));
        assertEquals(TOP_FINAL_CLIMB_POSE, result.plan().finalWaypoint());
    }

    @Test
    void planAutoClimb_rejectsWhenOnlyIncludedSideIsTooCloseAndOverrotated() {
        AutoPaths.AutoClimbPlanningResult result = AutoPaths.planAutoClimb(
            new Pose2d(1.25, 4.0, Rotation2d.fromDegrees(90.0)),
            createTarget(TOP_INCLUSION_ZONE, BOTTOM_INCLUSION_ZONE)
        );

        assertFalse(result.isAccepted());
        assertEquals(AutoPaths.AutoClimbRejectReason.TOO_CLOSE_AND_OVERROTATED, result.rejectReason());
    }

    @Test
    void planAutoClimb_selectsOtherSideWhenClosestSideIsTooCloseAndOverrotated() {
        AutoPaths.AutoClimbPlanningResult result = AutoPaths.planAutoClimb(
            new Pose2d(1.25, 4.0, Rotation2d.fromDegrees(90.0)),
            createTarget(SHARED_INCLUSION_ZONE, SHARED_INCLUSION_ZONE)
        );

        assertTrue(result.isAccepted());
        assertNotNull(result.plan());
        assertEquals("bottom_side", result.plan().side().name());
        assertEquals(BOTTOM_FINAL_CLIMB_POSE, result.plan().finalWaypoint());
    }
}
