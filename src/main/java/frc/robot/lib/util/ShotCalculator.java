package frc.robot.lib.util;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.lib.util.ballistics.BallisticsPhysics;
import java.util.function.DoubleUnaryOperator;

public class ShotCalculator {
    private static final double MIN_EFFECTIVE_COS = 0.05;
    private static final double MIN_FLIGHT_TIME_SECONDS = 1e-3;
    private static final double MIN_EXIT_VELOCITY_DENOMINATOR = 1e-6;
    private static final int FLIGHT_TIME_SOLVE_ITERATIONS = 4;
    private static final double FLIGHT_TIME_SOLVE_TOLERANCE_SECONDS = 1e-3;

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
        return calculate(
            targetLocation,
            shooterPosition,
            fieldRelativeSpeeds,
            lerpTable,
            latencyCompensationSeconds,
            measuredFlywheelRps,
            wheelRpsToExitVelocity,
            wheelRpsToSpinRateRadPerSec,
            shooterOffsetFromRobotCenter,
            robotHeading,
            true
        );
    }

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
        Rotation2d robotHeading,
        boolean useFlightTimeVelocitySolve
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
        double exitVelocity = calculateExitVelocityMetersPerSec(
            shooterLerpTableDistanceToTarget,
            hoodAngleDegrees,
            finalFlightTime,
            flywheelVelocityRPS,
            measuredFlywheelRps,
            wheelRpsToExitVelocity,
            wheelRpsToSpinRateRadPerSec,
            shooterPosition.getZ(),
            targetLocation.getZ(),
            useFlightTimeVelocitySolve
        );
        
        return new ShotData(
            new Rotation2d(shooterAngleToTarget),
            Rotation2d.fromDegrees(hoodAngleDegrees),
            flywheelVelocityRPS,
            exitVelocity,
            finalFlightTime,
            shooterLerpTableDistanceToTarget,
            new Translation2d(targetX, targetY)
        );
    }

    public static double calculateExitVelocityMetersPerSec(
        double distanceMeters,
        InterpolatingMatrixTreeMap<Double, N3, N1> lerpTable,
        double measuredFlywheelRps,
        DoubleUnaryOperator wheelRpsToExitVelocity,
        DoubleUnaryOperator wheelRpsToSpinRateRadPerSec,
        double shooterHeightMeters,
        double targetHeightMeters
    ) {
        return calculateExitVelocityMetersPerSec(
            distanceMeters,
            lerpTable,
            measuredFlywheelRps,
            wheelRpsToExitVelocity,
            wheelRpsToSpinRateRadPerSec,
            shooterHeightMeters,
            targetHeightMeters,
            true
        );
    }

    public static double calculateExitVelocityMetersPerSec(
        double distanceMeters,
        InterpolatingMatrixTreeMap<Double, N3, N1> lerpTable,
        double measuredFlywheelRps,
        DoubleUnaryOperator wheelRpsToExitVelocity,
        DoubleUnaryOperator wheelRpsToSpinRateRadPerSec,
        double shooterHeightMeters,
        double targetHeightMeters,
        boolean useFlightTimeVelocitySolve
    ) {
        double hoodAngleDegrees = lerpTable.get(distanceMeters).get(0, 0);
        double setpointFlywheelRps = lerpTable.get(distanceMeters).get(1, 0);
        double flightTimeSeconds = lerpTable.get(distanceMeters).get(2, 0);
        return calculateExitVelocityMetersPerSec(
            distanceMeters,
            hoodAngleDegrees,
            flightTimeSeconds,
            setpointFlywheelRps,
            measuredFlywheelRps,
            wheelRpsToExitVelocity,
            wheelRpsToSpinRateRadPerSec,
            shooterHeightMeters,
            targetHeightMeters,
            useFlightTimeVelocitySolve
        );
    }

    public static double calculateSpinRateRadPerSec(
        double distanceMeters,
        InterpolatingMatrixTreeMap<Double, N3, N1> lerpTable,
        double measuredFlywheelRps,
        DoubleUnaryOperator wheelRpsToExitVelocity,
        DoubleUnaryOperator wheelRpsToSpinRateRadPerSec,
        double shooterHeightMeters,
        double targetHeightMeters
    ) {
        return calculateSpinRateRadPerSec(
            distanceMeters,
            lerpTable,
            measuredFlywheelRps,
            wheelRpsToExitVelocity,
            wheelRpsToSpinRateRadPerSec,
            shooterHeightMeters,
            targetHeightMeters,
            true
        );
    }

    public static double calculateSpinRateRadPerSec(
        double distanceMeters,
        InterpolatingMatrixTreeMap<Double, N3, N1> lerpTable,
        double measuredFlywheelRps,
        DoubleUnaryOperator wheelRpsToExitVelocity,
        DoubleUnaryOperator wheelRpsToSpinRateRadPerSec,
        double shooterHeightMeters,
        double targetHeightMeters,
        boolean useFlightTimeVelocitySolve
    ) {
        double hoodAngleDegrees = lerpTable.get(distanceMeters).get(0, 0);
        double setpointFlywheelRps = lerpTable.get(distanceMeters).get(1, 0);
        double flightTimeSeconds = lerpTable.get(distanceMeters).get(2, 0);
        return calculateShotKinematics(
            distanceMeters,
            hoodAngleDegrees,
            flightTimeSeconds,
            setpointFlywheelRps,
            measuredFlywheelRps,
            wheelRpsToExitVelocity,
            wheelRpsToSpinRateRadPerSec,
            shooterHeightMeters,
            targetHeightMeters,
            useFlightTimeVelocitySolve
        ).spinRateRadPerSec();
    }

    public static double calculateExitVelocityMetersPerSec(
        double distanceMeters,
        InterpolatingMatrixTreeMap<Double, N3, N1> lerpTable,
        double measuredFlywheelRps,
        DoubleUnaryOperator wheelRpsToExitVelocity
    ) {
        return calculateExitVelocityMetersPerSec(
            distanceMeters,
            lerpTable,
            measuredFlywheelRps,
            wheelRpsToExitVelocity,
            ignoredRps -> 0.0,
            0.0,
            0.0
        );
    }

    private static double calculateExitVelocityMetersPerSec(
        double distanceMeters,
        double hoodAngleDegrees,
        double flightTimeSeconds,
        double setpointFlywheelRps,
        double measuredFlywheelRps,
        DoubleUnaryOperator wheelRpsToExitVelocity,
        DoubleUnaryOperator wheelRpsToSpinRateRadPerSec,
        double shooterHeightMeters,
        double targetHeightMeters,
        boolean useFlightTimeVelocitySolve
    ) {
        return calculateShotKinematics(
            distanceMeters,
            hoodAngleDegrees,
            flightTimeSeconds,
            setpointFlywheelRps,
            measuredFlywheelRps,
            wheelRpsToExitVelocity,
            wheelRpsToSpinRateRadPerSec,
            shooterHeightMeters,
            targetHeightMeters,
            useFlightTimeVelocitySolve
        ).exitVelocityMetersPerSec();
    }

    private static ShotKinematics calculateShotKinematics(
        double distanceMeters,
        double hoodAngleDegrees,
        double flightTimeSeconds,
        double setpointFlywheelRps,
        double measuredFlywheelRps,
        DoubleUnaryOperator wheelRpsToExitVelocity,
        DoubleUnaryOperator wheelRpsToSpinRateRadPerSec,
        double shooterHeightMeters,
        double targetHeightMeters,
        boolean useFlightTimeVelocitySolve
    ) {
        double safeSetpointRps = Math.max(MIN_EXIT_VELOCITY_DENOMINATOR, Math.abs(setpointFlywheelRps));
        if (safeSetpointRps <= MIN_EXIT_VELOCITY_DENOMINATOR) {
            return new ShotKinematics(0.0, 0.0);
        }

        double setpointExitVelocityMetersPerSec = wheelRpsToExitVelocity.applyAsDouble(setpointFlywheelRps);
        if (!Double.isFinite(setpointExitVelocityMetersPerSec)) {
            setpointExitVelocityMetersPerSec = 0.0;
        }

        double measuredScale = measuredFlywheelRps / safeSetpointRps;
        double setpointSpinRateRadPerSec = wheelRpsToSpinRateRadPerSec.applyAsDouble(setpointFlywheelRps);
        if (!Double.isFinite(setpointSpinRateRadPerSec)) {
            setpointSpinRateRadPerSec = 0.0;
        }

        if (!useFlightTimeVelocitySolve) {
            return new ShotKinematics(
                setpointExitVelocityMetersPerSec * measuredScale,
                setpointSpinRateRadPerSec * measuredScale
            );
        }

        double baseExitVelocity = solveSetpointExitVelocityMetersPerSec(
            distanceMeters,
            hoodAngleDegrees,
            flightTimeSeconds,
            setpointFlywheelRps,
            wheelRpsToExitVelocity,
            wheelRpsToSpinRateRadPerSec,
            shooterHeightMeters,
            targetHeightMeters
        );
        double solvedExitVelocity = baseExitVelocity * measuredScale;
        double baseSpinRateRadPerSec = scaleSpinForExitVelocity(
            setpointSpinRateRadPerSec,
            setpointExitVelocityMetersPerSec,
            baseExitVelocity
        );
        double solvedSpinRateRadPerSec = baseSpinRateRadPerSec * measuredScale;
        return new ShotKinematics(solvedExitVelocity, solvedSpinRateRadPerSec);
    }

    private static double solveSetpointExitVelocityMetersPerSec(
        double distanceMeters,
        double hoodAngleDegrees,
        double flightTimeSeconds,
        double setpointFlywheelRps,
        DoubleUnaryOperator wheelRpsToExitVelocity,
        DoubleUnaryOperator wheelRpsToSpinRateRadPerSec,
        double shooterHeightMeters,
        double targetHeightMeters
    ) {
        double hoodAngleRadians = Math.toRadians(hoodAngleDegrees);
        double effectiveCos = Math.max(MIN_EFFECTIVE_COS, Math.abs(Math.cos(hoodAngleRadians)));
        double targetFlightTime = Math.max(MIN_FLIGHT_TIME_SECONDS, flightTimeSeconds);
        double kinematicGuess = distanceMeters / (targetFlightTime * effectiveCos);
        double wheelModelSetpointExitVelocity = Math.abs(wheelRpsToExitVelocity.applyAsDouble(setpointFlywheelRps));
        double velocityGuess = Math.max(
            MIN_EXIT_VELOCITY_DENOMINATOR,
            Double.isFinite(wheelModelSetpointExitVelocity) && wheelModelSetpointExitVelocity > MIN_EXIT_VELOCITY_DENOMINATOR
                ? wheelModelSetpointExitVelocity
                : kinematicGuess
        );
        double setpointSpinRateRadPerSec = wheelRpsToSpinRateRadPerSec.applyAsDouble(setpointFlywheelRps);
        if (!Double.isFinite(setpointSpinRateRadPerSec)) {
            setpointSpinRateRadPerSec = 0.0;
        }

        for (int i = 0; i < FLIGHT_TIME_SOLVE_ITERATIONS; i++) {
            double spinRateRadPerSec = scaleSpinForExitVelocity(
                setpointSpinRateRadPerSec,
                wheelModelSetpointExitVelocity,
                velocityGuess
            );
            double estimatedFlightTime = BallisticsPhysics.estimateFlightTime(
                distanceMeters,
                hoodAngleRadians,
                velocityGuess,
                shooterHeightMeters,
                targetHeightMeters,
                spinRateRadPerSec
            );
            if (!Double.isFinite(estimatedFlightTime) || estimatedFlightTime <= MIN_FLIGHT_TIME_SECONDS) {
                break;
            }
            if (Math.abs(estimatedFlightTime - targetFlightTime) <= FLIGHT_TIME_SOLVE_TOLERANCE_SECONDS) {
                break;
            }

            double scale = estimatedFlightTime / targetFlightTime;
            scale = Math.max(0.5, Math.min(2.0, scale));
            velocityGuess = Math.max(MIN_EXIT_VELOCITY_DENOMINATOR, velocityGuess * scale);
        }

        return velocityGuess;
    }

    private static double scaleSpinForExitVelocity(
        double setpointSpinRateRadPerSec,
        double setpointExitVelocityMetersPerSec,
        double recalculatedExitVelocityMetersPerSec
    ) {
        double safeSetpointExitVelocity = Math.abs(setpointExitVelocityMetersPerSec);
        if (!Double.isFinite(safeSetpointExitVelocity) || safeSetpointExitVelocity < MIN_EXIT_VELOCITY_DENOMINATOR) {
            safeSetpointExitVelocity = MIN_EXIT_VELOCITY_DENOMINATOR;
        }

        double safeRecalculatedExitVelocity = Double.isFinite(recalculatedExitVelocityMetersPerSec)
            ? recalculatedExitVelocityMetersPerSec
            : 0.0;
        return setpointSpinRateRadPerSec * (safeRecalculatedExitVelocity / safeSetpointExitVelocity);
    }
}
