package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;
import frc.robot.lib.util.ShotCalculator;
import frc.robot.lib.util.ballistics.ProjectileVisualizer;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.shooter.Shooter;

public class VisualizeShot {
    public VisualizeShot() {
        this(Shooter.getInstance().getShotExitVelocityMetersPerSec());
    }

    public VisualizeShot(double launchExitVelocityMetersPerSec) {
        visualize(launchExitVelocityMetersPerSec);
    }

    private void visualize(double launchExitVelocityMetersPerSec) {
        if (Constants.currentMode != Constants.Mode.SIM) {
            return;
        }
        
        // Convert robot pose from field-relative Pose2d to Pose3d
        Pose3d robotPose3d = new Pose3d(RobotState.getInstance().getEstimatedPose());
        Rotation2d robotHeading = RobotState.getInstance().getEstimatedPose().getRotation();
        ChassisSpeeds fieldRelativeSpeeds = RobotState.getInstance().getFieldRelativeSpeeds();
        
        // Transform shooter relative pose to field coordinates
        Pose3d shooterPose = robotPose3d.plus(
            new Transform3d(new Pose3d(), Shooter.getInstance().getShooterRelativePose())
        );
        Translation2d shooterOffsetFromRobotCenter = Shooter.getInstance().getShooterRelativePose().getTranslation().toTranslation2d();

        // Apply hood angle and turret rotation as local rotations
        // This needs to be done by creating a proper rotation that combines:
        // 1. The robot's yaw (Z rotation) - already in shooterPose
        // 2. The turret's yaw (Z rotation) - needs to be added
        // 3. The hood's pitch (Y rotation) - needs to be added
        Rotation3d currentRotation = shooterPose.getRotation();
        double hoodAngleRadians = Shooter.getInstance().getHoodAngleRotations() * (2 * Math.PI);
        double turretAngleRadians = Shooter.getInstance().getTurretAngleRotations() * (2 * Math.PI);
        
        // Create new rotation: keep roll (X) at 0, add pitch (Y) from hood, combine yaw (Z) from robot + turret
        Rotation3d newRotation = new Rotation3d(
            0,  // roll (X)
            hoodAngleRadians,  // pitch (Y) from hood
            currentRotation.getZ() + turretAngleRadians  // yaw (Z) from robot rotation + turret rotation
        );
        
        shooterPose = new Pose3d(shooterPose.getTranslation(), newRotation);

        double flywheelRPS = Shooter.getInstance().getFlywheelVelocityRotationsPerSec();
        double exitVelocity = launchExitVelocityMetersPerSec;
        Superstructure superstructure = Superstructure.getInstance();
        Translation3d targetLocation = superstructure.getCurrentFieldTargetLocation();
        double shooterDistanceToTarget =
            shooterPose.getTranslation().toTranslation2d().getDistance(targetLocation.toTranslation2d());
        double backspinRadPerSec = ShotCalculator.calculateSpinRateRadPerSec(
            shooterDistanceToTarget,
            superstructure.getCurrentTargetLerpTable(),
            flywheelRPS,
            rps -> Shooter.getInstance().calculateBackSpinRPM(rps) * 2.0 * Math.PI / 60.0
        );
        double backspinRPM = backspinRadPerSec * 60.0 / (2.0 * Math.PI);
        
        // Include tangential velocity from robot rotation (omega cross r)
        double omega = fieldRelativeSpeeds.omegaRadiansPerSecond;
        double dx = shooterOffsetFromRobotCenter.getX();
        double dy = shooterOffsetFromRobotCenter.getY();

        double tangentialVxRobot = -omega * dy;
        double tangentialVyRobot = omega * dx;

        double cos = robotHeading.getCos();
        double sin = robotHeading.getSin();
        double tangentialVxField = tangentialVxRobot * cos - tangentialVyRobot * sin;
        double tangentialVyField = tangentialVxRobot * sin + tangentialVyRobot * cos;

        double shooterVxField = fieldRelativeSpeeds.vxMetersPerSecond + tangentialVxField;
        double shooterVyField = fieldRelativeSpeeds.vyMetersPerSecond + tangentialVyField;

        // Debug logging for shot parameters
        if (Constants.VERBOSE_LOGGING_ENABLED) {
            Logger.recordOutput("VisualizeShot/shooterPose", shooterPose);
            Logger.recordOutput("VisualizeShot/launchTime", Timer.getFPGATimestamp());
            Logger.recordOutput("VisualizeShot/hoodAngleDegrees", Math.toDegrees(hoodAngleRadians));
            Logger.recordOutput("VisualizeShot/turretAngleDegrees", Math.toDegrees(turretAngleRadians));
            Logger.recordOutput("VisualizeShot/totalYawDegrees", Math.toDegrees(newRotation.getZ()));
            Logger.recordOutput("VisualizeShot/exitVelocityMPS", exitVelocity);
            Logger.recordOutput("VisualizeShot/flywheelRPS", flywheelRPS);
            Logger.recordOutput("VisualizeShot/backspinRPM", backspinRPM);
            Logger.recordOutput("VisualizeShot/robotVx", fieldRelativeSpeeds.vxMetersPerSecond);
            Logger.recordOutput("VisualizeShot/robotVy", fieldRelativeSpeeds.vyMetersPerSecond);
            Logger.recordOutput("VisualizeShot/omegaRadPerSec", omega);
            Logger.recordOutput("VisualizeShot/tangentialVxField", tangentialVxField);
            Logger.recordOutput("VisualizeShot/tangentialVyField", tangentialVyField);
            Logger.recordOutput("VisualizeShot/shooterVxField", shooterVxField);
            Logger.recordOutput("VisualizeShot/shooterVyField", shooterVyField);
            Logger.recordOutput("VisualizeShot/targetLocation", targetLocation);
        }

        ProjectileVisualizer.addProjectile(
            shooterVxField,
            shooterVyField,
            exitVelocity,
            shooterPose,
            targetLocation.getZ(),
            backspinRPM
        );
    }
}
