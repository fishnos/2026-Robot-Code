package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;

public final class ZoneConstants {
    private ZoneConstants() {}

    public record RectangleZone(String name, Translation2d cornerA, Translation2d cornerB) {
        public RectangleZone {
            Objects.requireNonNull(name);
            Objects.requireNonNull(cornerA);
            Objects.requireNonNull(cornerB);
        }

        public double minX() {
            return Math.min(cornerA.getX(), cornerB.getX());
        }

        public double maxX() {
            return Math.max(cornerA.getX(), cornerB.getX());
        }

        public double minY() {
            return Math.min(cornerA.getY(), cornerB.getY());
        }

        public double maxY() {
            return Math.max(cornerA.getY(), cornerB.getY());
        }

        public boolean contains(Translation2d point) {
            double x = point.getX();
            double y = point.getY();
            return x >= minX() && x <= maxX() && y >= minY() && y <= maxY();
        }
    }

    public static final class Tower {
        private Tower() {}

        public static final RectangleZone EXCLUSION = new RectangleZone(
            "tower",
            new Translation2d(0.959, 4.324),
            new Translation2d(1.077, 3.156)
        );

        public static final RectangleZone LEFT = new RectangleZone(
            "tower_left",
            new Translation2d(1.500, 7.640),
            new Translation2d(3.456, 3.350)
        );

        public static final RectangleZone RIGHT = new RectangleZone(
            "tower_right",
            new Translation2d(1.500, 7.640),
            new Translation2d(3.456, 0.418)
        );
    }

    public static final class Alliance {
        private Alliance() {}

        public static final RectangleZone LOWER = new RectangleZone(
            "alliance_lower",
            new Translation2d(0.0, 0.0),
            new Translation2d(4.55, 1.24)
        );

        // Maintains rectangular decomposition through the 1.24->1.27 Y transition.
        public static final RectangleZone LOWER_TRANSITION = new RectangleZone(
            "alliance_lower_transition",
            new Translation2d(0.0, 1.24),
            new Translation2d(4.0, 1.27)
        );

        public static final RectangleZone MIDDLE = new RectangleZone(
            "alliance_middle",
            new Translation2d(0.0, 1.27),
            new Translation2d(4.0, 6.8)
        );

        public static final RectangleZone UPPER = new RectangleZone(
            "alliance_upper",
            new Translation2d(0.0, 6.8),
            new Translation2d(4.55, 8.1)
        );

        public static final List<RectangleZone> COMPOSITE = List.of(
            LOWER,
            LOWER_TRANSITION,
            MIDDLE,
            UPPER
        );
    }

    public static final class Hub {
        private Hub() {}

        public static final RectangleZone EXCLUSION = new RectangleZone(
            "hub_exclusion",
            new Translation2d(4.00, 4.6),
            new Translation2d(5.2, 3.45)
        );
    }

    public static final class Neutral {
        private Neutral() {}

        public static final RectangleZone LOWER = new RectangleZone(
            "neutral_lower",
            new Translation2d(4.7, 0.0),
            new Translation2d(11.85, 1.26)
        );

        // Thin transition band between the 1.26 and 1.3 breakpoints.
        public static final RectangleZone LOWER_TRANSITION = new RectangleZone(
            "neutral_lower_transition",
            new Translation2d(4.7, 1.26),
            new Translation2d(11.85, 1.3)
        );

        public static final RectangleZone MIDDLE = new RectangleZone(
            "neutral_middle",
            new Translation2d(5.2, 1.3),
            new Translation2d(11.33, 6.75)
        );

        // Thin transition band between the 6.75 and 6.8 breakpoints.
        public static final RectangleZone UPPER_TRANSITION = new RectangleZone(
            "neutral_upper_transition",
            new Translation2d(5.2, 6.75),
            new Translation2d(11.33, 6.8)
        );

        public static final RectangleZone UPPER = new RectangleZone(
            "neutral_upper",
            new Translation2d(4.7, 6.8),
            new Translation2d(11.85, 8.1)
        );

        public static final List<RectangleZone> COMPOSITE = List.of(
            LOWER,
            LOWER_TRANSITION,
            MIDDLE,
            UPPER_TRANSITION,
            UPPER
        );
    }

    public static final class OpposingAlliance {
        private OpposingAlliance() {}

        // Represents the "other side" of Alliance.COMPOSITE relative to the current alliance.
        // Use with ZoneUtil opposing-alliance helpers for automatic mirroring behavior.
        public static final List<RectangleZone> COMPOSITE = Alliance.COMPOSITE;
    }

    public static final Map<String, RectangleZone> ZONES = Map.ofEntries(
        Map.entry(Tower.EXCLUSION.name(), Tower.EXCLUSION),
        Map.entry(Tower.LEFT.name(), Tower.LEFT),
        Map.entry(Tower.RIGHT.name(), Tower.RIGHT),
        Map.entry(Alliance.LOWER.name(), Alliance.LOWER),
        Map.entry(Alliance.LOWER_TRANSITION.name(), Alliance.LOWER_TRANSITION),
        Map.entry(Alliance.MIDDLE.name(), Alliance.MIDDLE),
        Map.entry(Alliance.UPPER.name(), Alliance.UPPER),
        Map.entry(Hub.EXCLUSION.name(), Hub.EXCLUSION),
        Map.entry(Neutral.LOWER.name(), Neutral.LOWER),
        Map.entry(Neutral.LOWER_TRANSITION.name(), Neutral.LOWER_TRANSITION),
        Map.entry(Neutral.MIDDLE.name(), Neutral.MIDDLE),
        Map.entry(Neutral.UPPER_TRANSITION.name(), Neutral.UPPER_TRANSITION),
        Map.entry(Neutral.UPPER.name(), Neutral.UPPER)
    );

    public static Optional<RectangleZone> getZone(String name) {
        return Optional.ofNullable(ZONES.get(name));
    }
}
