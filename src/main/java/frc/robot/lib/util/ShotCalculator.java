package frc.robot.lib.util;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.function.DoubleUnaryOperator;

public class ShotCalculator {
    private static final double MIN_EXIT_VELOCITY_DENOMINATOR = 1e-6;

    private record ShotKinematics(double exitVelocityMetersPerSec, double spinRateRadPerSec) {}

    public record ShotData(
        Rotation2d targetFieldYaw,
        Rotation2d hoodPitch,
        double flywheelRPS,
        double exitVelocity,
        double flightTime,
        double effectiveDistance,
        Translation2d compensatedTargetPosition
    ) {}

    public static ShotData calculate(
        Translation3d targetLocation,
        Translation3d shooterPosition,
        ChassisSpeeds fieldRelativeSpeeds,
        InterpolatingMatrixTreeMap<Double, N3, N1> lerpTable,
        double latencyCompensationSeconds,
        double measuredFlywheelRps,
        DoubleUnaryOperator wheelRpsToExitVelocity,
        DoubleUnaryOperator wheelRpsToSpinRateRadPerSec,
        Translation2d shooterOffsetFromRobotCenter,
        Rotation2d robotHeading
    ) {
        // Total shooter velocity in field frame (linear + tangential from robot omega).
        double omega = fieldRelativeSpeeds.omegaRadiansPerSecond;
        double dx = shooterOffsetFromRobotCenter.getX();
        double dy = shooterOffsetFromRobotCenter.getY();

        // Tangential velocity in robot frame from omega x r.
        double tangentialVxRobot = -omega * dy;
        double tangentialVyRobot = omega * dx;

        // Rotate tangential component into field frame.
        double cosHeading = robotHeading.getCos();
        double sinHeading = robotHeading.getSin();
        double tangentialVxField = tangentialVxRobot * cosHeading - tangentialVyRobot * sinHeading;
        double tangentialVyField = tangentialVxRobot * sinHeading + tangentialVyRobot * cosHeading;

        double shooterVxField = fieldRelativeSpeeds.vxMetersPerSecond + tangentialVxField;
        double shooterVyField = fieldRelativeSpeeds.vyMetersPerSecond + tangentialVyField;
        

        // Iteratively solve for correct distance and flight time (robot reference frame approach)
        // Use HORIZONTAL distance (2D) as that's what the lerp table expects
        double shooterLerpTableDistanceToTarget = shooterPosition.toTranslation2d().getDistance(targetLocation.toTranslation2d());

        double shotFlightTime = 0.0;
        double targetX = targetLocation.getX();
        double targetY = targetLocation.getY();

        // Iterate to converge (robot frame: shot velocity is relative to robot, target appears to move)
        for (int i = 0; i < 20; i++) {
            // Get shooter flight time for current distance estimate
            shotFlightTime = lerpTable.get(shooterLerpTableDistanceToTarget).get(2, 0);

            // In robot frame, target appears to move. Predict where it will appear to be
            // Use total shooter velocity (includes tangential component from omega)
            double vxDisplacement = shotFlightTime * shooterVxField + latencyCompensationSeconds * shooterVxField;
            double vyDisplacement = shotFlightTime * shooterVyField + latencyCompensationSeconds * shooterVyField;

            targetX = targetLocation.getX() - vxDisplacement;
            targetY = targetLocation.getY() - vyDisplacement;

            // Recalculate distance to compensated target
            double oldDistance = shooterLerpTableDistanceToTarget;
            shooterLerpTableDistanceToTarget = shooterPosition.toTranslation2d().getDistance(new Translation2d(targetX, targetY));

            double distanceChange = Math.abs(shooterLerpTableDistanceToTarget - oldDistance);

            // Stop iterating if converged (distance change < 1cm)
            if (distanceChange < 0.01) {
                break;
            }
        }

        double shooterAngleToTarget = Math.atan2(targetY - shooterPosition.getY(), targetX - shooterPosition.getX());
        
        // Get final settings for this distance
        double hoodAngleDegrees = lerpTable.get(shooterLerpTableDistanceToTarget).get(0, 0);
        double flywheelVelocityRPS = lerpTable.get(shooterLerpTableDistanceToTarget).get(1, 0);
        double finalFlightTime = lerpTable.get(shooterLerpTableDistanceToTarget).get(2, 0);

        ShotKinematics shotKinematics = calculateShotKinematics(
            flywheelVelocityRPS,
            measuredFlywheelRps,
            wheelRpsToExitVelocity,
            wheelRpsToSpinRateRadPerSec
        );

        return new ShotData(
            new Rotation2d(shooterAngleToTarget),
            Rotation2d.fromDegrees(hoodAngleDegrees),
            flywheelVelocityRPS,
            shotKinematics.exitVelocityMetersPerSec(),
            finalFlightTime,
            shooterLerpTableDistanceToTarget,
            new Translation2d(targetX, targetY)
        );
    }

    public static double calculateExitVelocityMetersPerSec(
        double distanceMeters,
        InterpolatingMatrixTreeMap<Double, N3, N1> lerpTable,
        double measuredFlywheelRps,
        DoubleUnaryOperator wheelRpsToExitVelocity
    ) {
        double setpointFlywheelRps = lerpTable.get(distanceMeters).get(1, 0);
        return calculateShotKinematics(
            setpointFlywheelRps,
            measuredFlywheelRps,
            wheelRpsToExitVelocity,
            ignoredRps -> 0.0
        ).exitVelocityMetersPerSec();
    }

    public static double calculateSpinRateRadPerSec(
        double distanceMeters,
        InterpolatingMatrixTreeMap<Double, N3, N1> lerpTable,
        double measuredFlywheelRps,
        DoubleUnaryOperator wheelRpsToSpinRateRadPerSec
    ) {
        double setpointFlywheelRps = lerpTable.get(distanceMeters).get(1, 0);
        return calculateShotKinematics(
            setpointFlywheelRps,
            measuredFlywheelRps,
            ignoredRps -> 0.0,
            wheelRpsToSpinRateRadPerSec
        ).spinRateRadPerSec();
    }

    private static ShotKinematics calculateShotKinematics(
        double setpointFlywheelRps,
        double measuredFlywheelRps,
        DoubleUnaryOperator wheelRpsToExitVelocity,
        DoubleUnaryOperator wheelRpsToSpinRateRadPerSec
    ) {
        if (Math.abs(setpointFlywheelRps) <= MIN_EXIT_VELOCITY_DENOMINATOR) {
            return new ShotKinematics(0.0, 0.0);
        }

        double setpointExitVelocityMetersPerSec = wheelRpsToExitVelocity.applyAsDouble(setpointFlywheelRps);
        if (!Double.isFinite(setpointExitVelocityMetersPerSec)) {
            setpointExitVelocityMetersPerSec = 0.0;
        }

        double setpointSpinRateRadPerSec = wheelRpsToSpinRateRadPerSec.applyAsDouble(setpointFlywheelRps);
        if (!Double.isFinite(setpointSpinRateRadPerSec)) {
            setpointSpinRateRadPerSec = 0.0;
        }

        // Aiming uses hood/RPS/TOF from the table. Visualization-only shot kinematics are direct
        // wheel-model values scaled by measured flywheel speed versus the table setpoint.
        double measuredScale = measuredFlywheelRps / setpointFlywheelRps;
        if (!Double.isFinite(measuredScale)) {
            measuredScale = 0.0;
        }

        return new ShotKinematics(
            setpointExitVelocityMetersPerSec * measuredScale,
            setpointSpinRateRadPerSec * measuredScale
        );
    }
}
