package frc.robot.constants;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

class AlignmentConstantsTest {
    @Test
    void isWithinBounds_returnsTrueForTowerTopInteriorPoint() {
        Pair<Translation2d, Translation2d> bounds = AlignmentConstants.Tower.Top.BOUNDS;
        Pose2d interiorPose = poseAtMidpoint(bounds);
        assertTrue(AlignmentConstants.Tower.isWithinBounds(interiorPose, bounds));
    }

    @Test
    void isWithinBounds_returnsFalseForTowerTopExteriorPoint() {
        Pair<Translation2d, Translation2d> bounds = AlignmentConstants.Tower.Top.BOUNDS;
        Translation2d first = bounds.getFirst();
        Translation2d second = bounds.getSecond();
        double minY = Math.min(first.getY(), second.getY());
        Pose2d exteriorPose = new Pose2d(first.getX(), minY - 0.1, new Rotation2d());
        assertFalse(AlignmentConstants.Tower.isWithinBounds(exteriorPose, bounds));
    }

    private static Pose2d poseAtMidpoint(Pair<Translation2d, Translation2d> bounds) {
        Translation2d first = bounds.getFirst();
        Translation2d second = bounds.getSecond();
        return new Pose2d(
            (first.getX() + second.getX()) / 2.0,
            (first.getY() + second.getY()) / 2.0,
            new Rotation2d()
        );
    }
}
