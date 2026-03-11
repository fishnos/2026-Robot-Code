package frc.robot.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

class ShotKinematicsUtilTest {
    private static final double EPS = 1e-9;

    @Test
    void calculateShooterPosition_appliesRobotPoseAndShooterOffset() {
        Pose2d robotPose = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(90.0));
        Pose3d shooterRelativePose = new Pose3d(
            new Translation3d(1.0, 0.0, 0.5),
            new Rotation3d()
        );

        Translation3d shooterPosition = ShotKinematicsUtil.calculateShooterPosition(robotPose, shooterRelativePose);

        assertEquals(2.0, shooterPosition.getX(), EPS);
        assertEquals(4.0, shooterPosition.getY(), EPS);
        assertEquals(0.5, shooterPosition.getZ(), EPS);
    }

    @Test
    void calculateShooterFieldVelocity_includesTangentialContributionFromOmega() {
        ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(1.0, 2.0, 3.0);
        Translation2d shooterOffset = new Translation2d(0.2, -0.1);
        Rotation2d robotHeading = Rotation2d.fromDegrees(30.0);

        ShotKinematicsUtil.ShooterFieldVelocity velocity = ShotKinematicsUtil.calculateShooterFieldVelocity(
            fieldRelativeSpeeds,
            shooterOffset,
            robotHeading
        );

        double tangentialVxRobot = -fieldRelativeSpeeds.omegaRadiansPerSecond * shooterOffset.getY();
        double tangentialVyRobot = fieldRelativeSpeeds.omegaRadiansPerSecond * shooterOffset.getX();
        double expectedVx = fieldRelativeSpeeds.vxMetersPerSecond
            + tangentialVxRobot * robotHeading.getCos() - tangentialVyRobot * robotHeading.getSin();
        double expectedVy = fieldRelativeSpeeds.vyMetersPerSecond
            + tangentialVxRobot * robotHeading.getSin() + tangentialVyRobot * robotHeading.getCos();

        assertEquals(expectedVx, velocity.vxMetersPerSecond(), EPS);
        assertEquals(expectedVy, velocity.vyMetersPerSecond(), EPS);
    }

    @Test
    void calculateShooterKinematics_combinesPositionAndVelocityCalculations() {
        Pose2d robotPose = new Pose2d(1.5, -0.5, Rotation2d.fromDegrees(-20.0));
        Pose3d shooterRelativePose = new Pose3d(
            new Translation3d(0.3, 0.2, 0.6),
            new Rotation3d()
        );
        ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(0.5, -0.8, 1.2);

        ShotKinematicsUtil.ShooterKinematics kinematics = ShotKinematicsUtil.calculateShooterKinematics(
            robotPose,
            shooterRelativePose,
            fieldRelativeSpeeds
        );
        Translation3d expectedPosition = ShotKinematicsUtil.calculateShooterPosition(robotPose, shooterRelativePose);
        ShotKinematicsUtil.ShooterFieldVelocity expectedVelocity = ShotKinematicsUtil.calculateShooterFieldVelocity(
            fieldRelativeSpeeds,
            shooterRelativePose.getTranslation().toTranslation2d(),
            robotPose.getRotation()
        );

        assertEquals(expectedPosition.getX(), kinematics.shooterPosition().getX(), EPS);
        assertEquals(expectedPosition.getY(), kinematics.shooterPosition().getY(), EPS);
        assertEquals(expectedPosition.getZ(), kinematics.shooterPosition().getZ(), EPS);
        assertEquals(expectedVelocity.vxMetersPerSecond(), kinematics.shooterVxField(), EPS);
        assertEquals(expectedVelocity.vyMetersPerSecond(), kinematics.shooterVyField(), EPS);
    }
}
