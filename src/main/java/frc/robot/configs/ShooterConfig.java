package frc.robot.configs;

import java.util.List;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class ShooterConfig {
    public static class LerpEntry {
        public double distanceMeters;
        public double hoodAngleDegrees;
        public double flywheelVelocityRPS;
        public double flightTimeSeconds;
    }

    public List<LerpEntry> shootingLerpTable;
    // Backward-compatible alias for older config files.
    public List<LerpEntry> lerpTable;
    public List<LerpEntry> passLerpTable;

    public double minShotDistFromShooterMeters;
    public double maxShotDistFromShooterMeters;
    public double minPassDistFromShooterMeters;
    public double maxPassDistFromShooterMeters;
    public double latencyCompensationSeconds;

    public String canBusName;

    public int hoodCanId;
    public boolean isHoodInverted;

    public double hoodStatorCurrentLimit;
    public double hoodPeakForwardTorqueCurrent;
    public double hoodPeakReverseTorqueCurrent;

    public double hoodKS;
    public double hoodKV;
    public double hoodKA;
    public double hoodKP;
    public double hoodKI;
    public double hoodKD;

    public double hoodMotorToOutputShaftRatio;

    public double hoodStartingAngleDegrees;
    public double hoodMinAngleDegrees;
    public double hoodMaxAngleDegrees;

    public int turretCanId;
    public boolean isTurretInverted;

    public double turretStatorCurrentLimit;
    public double turretPeakForwardTorqueCurrent;
    public double turretPeakReverseTorqueCurrent;

    public double turretKS;
    public double turretKV;
    public double turretKA;
    public double turretKP;
    public double turretKI;
    public double turretKD;

    public double turretMotorToOutputShaftRatio;

    public double turretStartingAngleDeg;
    public double turretMinAngleDeg;
    public double turretMaxAngleDeg;

    public double turretMaxVelocityDegPerSec;
    public double turretMaxAccelerationDegPerSec2;

    public int flywheelCanId;
    public int flywheelFollowerCanId;
    public boolean isFlywheelInverted;
    public boolean isFlywheelFollowerOppositeDirection;

    public double flywheelStatorCurrentLimit;
    public double flywheelPeakForwardTorqueCurrent;
    public double flywheelPeakReverseTorqueCurrent;

    public double flywheelKS;
    public double flywheelKV;
    public double flywheelKA;
    public double flywheelKP;
    public double flywheelKI;
    public double flywheelKD;

    public double flywheelMotorToOutputShaftRatio;
    public double flywheelRadiusMeters;
    public double backRollerGearRatio;
    public double backRollerRadiusMeters;

    public double shooterPoseX;
    public double shooterPoseY;
    public double shooterPoseZ;
    public double shooterPoseRollDeg;
    public double shooterPosePitchDeg;
    public double shooterPoseYawDeg;

    public double hoodAngleToleranceDegrees;
    public double turretAngleToleranceRotations;
    public double flywheelVelocityToleranceRPS;

    public InterpolatingMatrixTreeMap<Double, N3, N1> getLerpTable() {
        return buildLerpTable(getShootingLerpEntries());
    }

    public InterpolatingMatrixTreeMap<Double, N3, N1> getPassLerpTable() {
        return buildLerpTable(passLerpTable);
    }

    public List<LerpEntry> getShootingLerpEntries() {
        if (shootingLerpTable != null && !shootingLerpTable.isEmpty()) {
            return shootingLerpTable;
        }
        return lerpTable;
    }

    private InterpolatingMatrixTreeMap<Double, N3, N1> buildLerpTable(List<LerpEntry> entries) {
        InterpolatingMatrixTreeMap<Double, N3, N1> table = new InterpolatingMatrixTreeMap<>();
        if (entries == null) {
            return table;
        }
        for (LerpEntry entry : entries) {
            table.put(entry.distanceMeters, new Matrix<N3, N1>(Nat.N3(), Nat.N1(), new double[]{
                entry.hoodAngleDegrees,
                entry.flywheelVelocityRPS,
                entry.flightTimeSeconds
            }));
        }
        return table;
    }

    public Pose3d getShooterPose3d() {
        return new Pose3d(
            new Translation3d(shooterPoseX, shooterPoseY, shooterPoseZ),
            new Rotation3d(
                Math.toRadians(shooterPoseRollDeg),
                Math.toRadians(shooterPosePitchDeg),
                Math.toRadians(shooterPoseYawDeg)
            )
        );
    }
}
