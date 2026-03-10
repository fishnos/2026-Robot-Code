// Copyright (c) 2026 FRC 2638
// http://github.com/rebels2638
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;

/**
 * Contains various field dimensions and useful reference points for the 2026
 * REBUILT game.
 * All units are in meters and poses have a blue alliance origin.
 * Currently using 2026 REBUILT official specifications.
 */
public class FieldConstants {
    public static final Translation3d kSHOOTER_TARGET = new Translation3d(49.85, 50.0, 0.46);
    public static final double fieldLength = Units.inchesToMeters(651.22);
    public static final double fieldWidth = Units.inchesToMeters(317.69);

    public static final double startingLineX = Units.inchesToMeters(156.61);
    public static final double fuelDiameter = Units.inchesToMeters(5.91);

    public static class Outpost {
        public static final Pose2d outpost = new Pose2d(
                Units.inchesToMeters(0.0),
                Units.inchesToMeters(34.72),
                Rotation2d.fromDegrees(0.0));
    }

    public static class Tower {
        // left to right (facing the tower)
        public static final Pose2d[] towerPoses = {
            new Pose2d(
                Units.inchesToMeters(59.76),
                Units.inchesToMeters(133.2),
                Rotation2d.fromDegrees(180.0)),
            new Pose2d(
                Units.inchesToMeters(59.76),
                Units.inchesToMeters(148.094),
                Rotation2d.fromDegrees(180.0)),
            new Pose2d(
                Units.inchesToMeters(59.76),
                Units.inchesToMeters(163.0),
                Rotation2d.fromDegrees(180.0))
        };

        public static final double towerWidth = Units.inchesToMeters(47.0);

        public enum TowerLevel {
            LOW(Units.inchesToMeters(27.0)),
            MID(Units.inchesToMeters(45.0)),
            HIGH(Units.inchesToMeters(63.0));

            TowerLevel(double height) {
                this.height = height;
            }

            public final double height;
        }
    }

    public static class Depot {
        public static final Translation2d[] depots = {
                new Translation2d(Units.inchesToMeters(0.0), Units.inchesToMeters(290.0)),
                new Translation2d(Units.inchesToMeters(0.0), Units.inchesToMeters(25.0))
        };
    }

    public static class Hub {
        public static final Translation3d hubCenter = new Translation3d(
                Units.inchesToMeters(182.105),
                Units.inchesToMeters(158.845),
                Units.inchesToMeters(71.0)); // receded inside of the hub hood 

        public static final double sideLength = Units.inchesToMeters(47.5);
        public static final double topHeight = Units.inchesToMeters(44.25);
    }

    public static class AllianceBounds {
        public static final double blueZoneLineX = Units.inchesToMeters(182.11);
        public static final double redZoneLineX = FieldConstants.fieldLength - blueZoneLineX;
    }

    public static class Bump {
        public static final double width = Units.inchesToMeters(73.0);
        public static final double depth = Units.inchesToMeters(44.4);
        public static final double height = Units.inchesToMeters(6.513);
    }

    public static class Trench {
        public static final double armWidth = Units.inchesToMeters(73.0);
        public static final double armHeight = Units.inchesToMeters(35.0);
    }

    public static class NeutralZone {
        public static final double boxWidth = Units.inchesToMeters(206.0);
        public static final double boxDepth = Units.inchesToMeters(72.0);
        public static final Translation2d boxCenter =
            new Translation2d(
                FieldConstants.fieldLength / 2.0,
                FieldConstants.fieldWidth / 2.0
            );
    }

    public static final double aprilTagWidth = Units.inchesToMeters(8.125);
    public static final int aprilTagCount = 32;
}
