package frc.robot.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class ShotKinematicsUtil {
    private ShotKinematicsUtil() {}

    public record ShooterFieldVelocity(
        double vxMetersPerSecond,
        double vyMetersPerSecond
    ) {}

    public record ShooterKinematics(
        Translation3d shooterPosition,
        Translation2d shooterOffsetFromRobotCenter,
        Rotation2d robotHeading,
        ChassisSpeeds fieldRelativeSpeeds,
        double shooterVxField,
        double shooterVyField
    ) {}

    public static ShooterKinematics calculateShooterKinematics(
        Pose2d robotPose,
        Pose3d shooterRelativePose,
        ChassisSpeeds fieldRelativeSpeeds
    ) {
        Translation3d shooterPosition = calculateShooterPosition(robotPose, shooterRelativePose);
        Translation2d shooterOffsetFromRobotCenter = shooterRelativePose.getTranslation().toTranslation2d();
        Rotation2d robotHeading = robotPose.getRotation();
        ShooterFieldVelocity shooterFieldVelocity = calculateShooterFieldVelocity(
            fieldRelativeSpeeds,
            shooterOffsetFromRobotCenter,
            robotHeading
        );

        return new ShooterKinematics(
            shooterPosition,
            shooterOffsetFromRobotCenter,
            robotHeading,
            fieldRelativeSpeeds,
            shooterFieldVelocity.vxMetersPerSecond(),
            shooterFieldVelocity.vyMetersPerSecond()
        );
    }

    public static Translation3d calculateShooterPosition(
        Pose2d robotPose,
        Pose3d shooterRelativePose
    ) {
        Pose3d robotPose3d = new Pose3d(
            new Translation3d(robotPose.getX(), robotPose.getY(), 0.0),
            new Rotation3d(0.0, 0.0, robotPose.getRotation().getRadians())
        );
        return robotPose3d.plus(new Transform3d(new Pose3d(), shooterRelativePose)).getTranslation();
    }

    public static ShooterFieldVelocity calculateShooterFieldVelocity(
        ChassisSpeeds fieldRelativeSpeeds,
        Translation2d shooterOffsetFromRobotCenter,
        Rotation2d robotHeading
    ) {
        double omega = fieldRelativeSpeeds.omegaRadiansPerSecond;
        double dx = shooterOffsetFromRobotCenter.getX();
        double dy = shooterOffsetFromRobotCenter.getY();

        double tangentialVxRobot = -omega * dy;
        double tangentialVyRobot = omega * dx;

        double tangentialVxField = tangentialVxRobot * robotHeading.getCos() - tangentialVyRobot * robotHeading.getSin();
        double tangentialVyField = tangentialVxRobot * robotHeading.getSin() + tangentialVyRobot * robotHeading.getCos();
        double shooterVxField = fieldRelativeSpeeds.vxMetersPerSecond + tangentialVxField;
        double shooterVyField = fieldRelativeSpeeds.vyMetersPerSecond + tangentialVyField;

        return new ShooterFieldVelocity(shooterVxField, shooterVyField);
    }
}
