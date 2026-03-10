package frc.robot.lib.util.ballistics;

public record TrajectoryResult(
    double flightTime,
    double finalHeight,
    double finalDistance,
    boolean reachedTarget
) {
}
