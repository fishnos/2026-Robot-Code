package frc.robot.lib.util.ballistics;

import frc.robot.constants.FieldConstants;

public final class BallisticsConstants {
    public static final double GRAVITY = 9.81;
    public static final double BALL_MASS = 0.213;
    public static final double BALL_RADIUS = FieldConstants.fuelDiameter / 2.0;
    public static final double CROSS_SECTIONAL_AREA = Math.PI * BALL_RADIUS * BALL_RADIUS;
    public static final double AIR_DENSITY = 1.225;
    public static final double DRAG_COEFFICIENT = 0.45;
    public static final double MAX_LIFT_COEFFICIENT = 0.35;

    private BallisticsConstants() {
    }
}
