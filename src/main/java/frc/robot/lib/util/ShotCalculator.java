package frc.robot.lib.util;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import java.util.function.DoubleUnaryOperator;

import frc.robot.lib.util.ballistics.BallisticsPhysics;

public class ShotCalculator {

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
        InterpolatingMatrixTreeMap<Double, N2, N1> lerpTable,
        double latencyCompensationSeconds,
        DoubleUnaryOperator rpsToExitVelocity,
        DoubleUnaryOperator rpsToSpinRateRadPerSec,
        Translation2d shooterOffsetFromRobotCenter,
        Rotation2d robotHeading
    ) {
        // Total shooter velocity in field frame (linear only)
        double shooterVxField = fieldRelativeSpeeds.vxMetersPerSecond;
        double shooterVyField = fieldRelativeSpeeds.vyMetersPerSecond;
        

        // Iteratively solve for correct distance and flight time (robot reference frame approach)
        // Use HORIZONTAL distance (2D) as that's what the lerp table expects
        double shooterDistanceToTarget = shooterPosition.toTranslation2d().getDistance(targetLocation.toTranslation2d());
        
        double shotFlightTime = 0.0;
        double targetX = targetLocation.getX();
        double targetY = targetLocation.getY();
        double targetHeight = targetLocation.getZ();
        
        // Iterate to converge (robot frame: shot velocity is relative to robot, target appears to move)
        for (int i = 0; i < 30; i++) {
            
            // Get shooter settings for current distance estimate
            double hoodAngleRotations = lerpTable.get(shooterDistanceToTarget).get(0, 0);
            double flywheelRPS = lerpTable.get(shooterDistanceToTarget).get(1, 0);
            double exitVelocity = rpsToExitVelocity.applyAsDouble(flywheelRPS);
            double launchAngle = hoodAngleRotations * 2 * Math.PI; // Convert to radians
            
            double shooterHeight = shooterPosition.getZ();
            double deltaHeight = targetHeight - shooterHeight;

            double spinRateRadPerSec = rpsToSpinRateRadPerSec.applyAsDouble(flywheelRPS);
            shotFlightTime = BallisticsPhysics.estimateFlightTime(
                shooterDistanceToTarget,
                launchAngle,
                exitVelocity,
                shooterHeight,
                shooterHeight + deltaHeight,
                spinRateRadPerSec
            );
            
            // In robot frame, target appears to move. Predict where it will appear to be
            // Use total shooter velocity (includes tangential component from omega)
            double vxDisplacement = shotFlightTime * shooterVxField + latencyCompensationSeconds * shooterVxField;
            double vyDisplacement = shotFlightTime * shooterVyField + latencyCompensationSeconds * shooterVyField;
            
            targetX = targetLocation.getX() - vxDisplacement;
            targetY = targetLocation.getY() - vyDisplacement;
            
            // Recalculate distance to compensated target
            double oldDistance = shooterDistanceToTarget;
            shooterDistanceToTarget = shooterPosition.toTranslation2d().getDistance(new Translation2d(targetX, targetY));
            
            double distanceChange = Math.abs(shooterDistanceToTarget - oldDistance);
            
            // Stop iterating if converged (distance change < 1cm)
            if (distanceChange < 0.01) {
                break;
            }
        }


        
        double shooterAngleToTarget = Math.atan2(targetY - shooterPosition.getY(), targetX - shooterPosition.getX());
        
        // Get final settings for this distance
        double hoodAngleRotations = lerpTable.get(shooterDistanceToTarget).get(0, 0);
        double flywheelVelocityRPS = lerpTable.get(shooterDistanceToTarget).get(1, 0);
        double exitVelocity = rpsToExitVelocity.applyAsDouble(flywheelVelocityRPS);
        
        return new ShotData(
            new Rotation2d(shooterAngleToTarget),
            new Rotation2d(hoodAngleRotations * 2 * Math.PI),
            flywheelVelocityRPS,
            exitVelocity,
            shotFlightTime,
            shooterDistanceToTarget,
            new Translation2d(targetX, targetY)
        );
    }
}
