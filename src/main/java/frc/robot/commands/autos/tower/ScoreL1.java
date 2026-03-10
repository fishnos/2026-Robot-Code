// package frc.robot.commands.autos.tower;

// import java.util.ArrayList;
// import java.util.List;
// import java.util.Optional;

// import org.littletonrobotics.junction.Logger;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.RobotState;
// import frc.robot.constants.Constants;
// import frc.robot.constants.FieldConstants;
// import frc.robot.lib.BLine.Path.PathConstraints;
// import frc.robot.lib.BLine.FlippingUtil;
// import frc.robot.lib.BLine.Path;
// import frc.robot.lib.BLine.Path.TranslationTarget;
// import frc.robot.lib.BLine.Path.Waypoint;
// import frc.robot.lib.util.RebelUtil;
// import frc.robot.subsystems.swerve.SwerveDrive;
// import frc.robot.subsystems.swerve.SwerveDrive.CurrentSystemState;
// import frc.robot.subsystems.swerve.SwerveDrive.DesiredSystemState;

// public class ScoreL1 extends Command {
//     private final SwerveDrive swerveDrive;
//     private final RobotState robotState;
//     private boolean insideAllianceBounds = true;

//     public ScoreL1() {
//         this.swerveDrive = SwerveDrive.getInstance();
//         this.robotState = RobotState.getInstance();
//     }

//     @Override
//     public void initialize() {
//         Pose2d currentPose = robotState.getEstimatedPose();

//         double blueZoneX = FieldConstants.AllianceBounds.blueZoneLineX;
//         double redZoneX = FieldConstants.AllianceBounds.redZoneLineX;

//         insideAllianceBounds = Constants.shouldFlipPath() ? (currentPose.getX() > redZoneX) : (currentPose.getX() < blueZoneX);
//         if (!insideAllianceBounds) {
//             return;
//         }

//         Logger.recordOutput("ScoreL1/Status", "Running");

//         Pose2d currentPoseBlue = FlippingUtil.flipFieldPose(currentPose);
//         Pose2d towerPoseBlue = Constants.AlignmentConstants.TOWER_WAYPOINTS[0];
//         double minDistanceToTower = towerPoseBlue.getTranslation().getDistance(currentPoseBlue.getTranslation());
//         int towerPoseIndex = 0;

//         for (int i = 0; i < Constants.AlignmentConstants.TOWER_WAYPOINTS.length; i++) {
//             Pose2d pose = Constants.AlignmentConstants.TOWER_WAYPOINTS[i];
//             double distance = pose.getTranslation().getDistance(currentPoseBlue.getTranslation());

//             if (distance < minDistanceToTower) {
//                 minDistanceToTower = distance;

//                 towerPoseBlue = pose;
//                 towerPoseIndex = i;
//             }
//         }

//         List<Path.PathElement> pathElements = new ArrayList<>();
//         pathElements.add(new Waypoint(currentPoseBlue));

//         boolean needsIntermediate = Constants.AlignmentConstants.shouldUseIntermediate(currentPoseBlue, towerPoseBlue);
//         Translation2d[] intermediateTranslation = Constants.AlignmentConstants.intermediateTranslation(
//             currentPoseBlue,
//             towerPoseIndex
//         );

//         if (needsIntermediate) {
//             for (int i = intermediateTranslation.length - 1; i >= 0; i--) {
//                 pathElements.add(new TranslationTarget(intermediateTranslation[i]));
//             }
//         }

//         pathElements.add(new Waypoint(towerPoseBlue, 0.08));

//         int finalTranslationOrdinal = pathElements.size() - 2;
//         PathConstraints constraints = new PathConstraints()
//                 .setMaxVelocityMetersPerSec(
//                     new Path.RangedConstraint(
//                         Constants.AlignmentConstants.APPROACH_MAX_VELOCITY_METERS_PER_SEC,
//                         0,
//                         finalTranslationOrdinal),
//                     new Path.RangedConstraint(
//                         Constants.AlignmentConstants.MAX_VELOCITY_METERS_PER_SEC,
//                         finalTranslationOrdinal,
//                         pathElements.size() - 1));
//         Path toClimb = new Path(pathElements, constraints);

//         swerveDrive.setPathSupplier(() -> toClimb, () -> false);
//         swerveDrive.setDesiredSystemState(DesiredSystemState.FOLLOW_PATH);
//     }

//     @Override
//     public void execute() {

//     }

//     @Override
//     public void end(boolean interrupted) {
//         Logger.recordOutput("ScoreL1/End/Interrupted", interrupted);
//     }

//     @Override
//     public boolean isFinished() {
//         return swerveDrive.getCurrentSystemState() == CurrentSystemState.IDLE || !insideAllianceBounds;
//     }
// }
