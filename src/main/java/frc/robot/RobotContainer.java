package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.AlignmentConstants;
// import frc.robot.commands.autos.tower.ScoreL1;
import frc.robot.constants.Constants;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.EventTrigger;
import frc.robot.lib.BLine.Path.PathConstraints;
import frc.robot.lib.BLine.Path.Waypoint;
import frc.robot.lib.input.XboxController;
import frc.robot.lib.util.ballistics.ProjectileVisualizer;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.TargetState;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.Hopper.HopperSetpoint;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.SwerveDrive;
// import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision;

//: TODO: make sure all lisences in tuner are activated
public class RobotContainer {
    private static final double DRIVE_HELPER_TRIGGER_THRESHOLD = 0.5;

    public static RobotContainer instance = null;

    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }

    private final XboxController xboxTester;
    private final XboxController xboxDriver;
    private final XboxController xboxOperator;

    private final RobotState robotState = RobotState.getInstance();
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final Vision vision = Vision.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Hopper hopper = Hopper.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Superstructure superstructure = Superstructure.getInstance();
    private Superstructure.DesiredIntakeState competitionDriveIntakeReleaseState =
        Superstructure.DesiredIntakeState.DEPLOYED;

    LoggedNetworkNumber shooterFlywheelSet = new LoggedNetworkNumber("flywheelSet", 0);
    LoggedNetworkNumber turret = new LoggedNetworkNumber("turretSet", 180);
    LoggedNetworkNumber hood = new LoggedNetworkNumber("hoodSet", 70);

    @SuppressWarnings("unused")
    private final ProjectileVisualizer projectileVisualizer = ProjectileVisualizer.getInstance();


    private RobotContainer() {
        registerEventTriggers();

        this.xboxTester = new XboxController(1);
        this.xboxOperator = new XboxController(2);
        this.xboxDriver = new XboxController(3);

        // Configure field-relative teleop input suppliers for SwerveDrive FSM.
        swerveDrive.setFieldRelativeTeleopInputSuppliers(
            () -> -MathUtil.applyDeadband(xboxDriver.getLeftY(), Constants.OperatorConstants.LEFT_Y_DEADBAND),
            () -> -MathUtil.applyDeadband(xboxDriver.getLeftX(), Constants.OperatorConstants.LEFT_X_DEADBAND),
            () -> -MathUtil.applyDeadband(xboxDriver.getRightX(), Constants.OperatorConstants.RIGHT_X_DEADBAND)
        );

        // Configure robot-relative teleop input suppliers for SwerveDrive FSM.
        swerveDrive.setRobotRelativeTeleopInputSuppliers(
            () -> -MathUtil.applyDeadband(xboxDriver.getLeftY(), Constants.OperatorConstants.LEFT_Y_DEADBAND),
            () -> -MathUtil.applyDeadband(xboxDriver.getLeftX(), Constants.OperatorConstants.LEFT_X_DEADBAND),
            () -> -MathUtil.applyDeadband(xboxDriver.getRightX(), Constants.OperatorConstants.RIGHT_X_DEADBAND)
        );
        swerveDrive.setRobotRelativeTeleopHeadingOffset(Rotation2d.fromDegrees(25.0));

        // Default teleop drive mode is field-relative.
        swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.TELOP_FIELD_RELATIVE);
        superstructure.setDesiredTargetState(TargetState.HUB);
        
        // Set default superstructure state to HOME
        superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.HOME);
        superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.STOWED);
        superstructure.setDesiredClimbState(Superstructure.DesiredClimbState.RETRACTED);

        // superstructure.setDesiredTargetState(Superstructure.TargetState.PASS_ALLIANCE_TOP);

        

        configureBindings();
    }

    private void registerEventTriggers() {
        FollowPath.registerEventTrigger("prepare_for_shot", new InstantCommand(() -> {
            superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.READY_FOR_SHOT);
        }));
        FollowPath.registerEventTrigger("shoot", () -> {
            superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.SHOOTING);
        });
    }

    private void configureBindings() {
        bindCompetitionDriveHelpers();

        // xboxDriver.getXButton().onTrue(
        //     new InstantCommand(() -> robotState.resetPose(new Pose2d(robotState.getEstimatedPose().getTranslation(), new Rotation2d(0))))
        // );
        // xboxDriver.getAButton().onTrue(
        //     new InstantCommand(() -> Shooter.getInstance().setHoodAngle(Rotation2d.fromDegrees(hood.getAsDouble())))
        // );
        // xboxDriver.getBButton().onTrue(
        //     new InstantCommand(() -> Shooter.getInstance().setShotVelocity(shooterFlywheelSet.getAsDouble()))
        // );
        // xboxDriver.getXButton().onTrue(
        //     new InstantCommand(() -> Hopper.getInstance().setSetpoint(HopperSetpoint.FEEDING))
        // );
        // xboxDriver.getYButton().onTrue(
        //     new InstantCommand(() -> Shooter.getInstance().setHoodAngle(Rotation2d.fromDegrees(70))).andThen(
        //         new InstantCommand(() -> Shooter.getInstance().setShotVelocity(0))
        //     ).andThen(
        //         new InstantCommand(() -> Hopper.getInstance().setSetpoint(HopperSetpoint.OFF))
        //     )
        // );

        // xboxDriver.getBButton().onTrue(
        //     new InstantCommand(() -> superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.SHOOTING))
        // );
        // xboxDriver.getXButton().onTrue(
        //     new InstantCommand(() -> superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.TRACKING))
        // );
        // );

        // xboxDriver.getAButton().onTrue(
        //     new InstantCommand(() -> shooter.setTurretAngle(new Rotation2d(180
        //     )))
        // );
        // xboxDriver.getBButton().onTrue(
        //     new InstantCommand(() -> shooter.setTurretAngle((Rotation2d.fromDegrees(turret.getAsDouble()))))
        // );

        // xboxDriver.getXButton().onTrue(
        //     new InstantCommand(() -> hopper.setSetpoint(HopperSetpoint.FEEDING))
        // );
        // xboxDriver.getYButton().onTrue(
        //     new InstantCommand(() -> hopper.setSetpoint(HopperSetpoint.OFF))
        // );
                // xboxOperator.getYButton().onTrue(
        //     new InstantCommand(() -> superstructure.setDesiredTargetState(Superstructure.TargetState.HUB))
        // );
        // xboxOperator.getUpDpad().onTrue(
        //     new InstantCommand(() -> superstructure.setDesiredTargetState(Superstructure.TargetState.PASS_ALLIANCE_TOP))
        // );
        // xboxOperator.getRightDpad().onTrue(
        //     new InstantCommand(() -> superstructure.setDesiredTargetState(Superstructure.TargetState.PASS_ALLIANCE_CENTER))
        // );
        // xboxOperator.getDownDpad().onTrue(
        //     new InstantCommand(() -> superstructure.setDesiredTargetState(Superstructure.TargetState.PASS_ALLIANCE_BOTTOM))
        // );
        // xboxOperator.getLeftBumper().onTrue(
        //     new InstantCommand(() -> superstructure.setDesiredTargetState(Superstructure.TargetState.PASS_NEUTRAL_TOP))
        // );
        // xboxOperator.getLeftDpad().onTrue(
        //     new InstantCommand(() -> superstructure.setDesiredTargetState(Superstructure.TargetState.PASS_NEUTRAL_CENTER))
        // );
        // xboxOperator.getRightBumper().onTrue(
        //     new InstantCommand(() -> superstructure.setDesiredTargetState(Superstructure.TargetState.PASS_NEUTRAL_BOTTOM))
        // );

        // Split-state pattern example:
        // superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.TRACKING);
        // superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.INTAKING);

        // xboxDriver.getAButton().onTrue(
        //     new InstantCommand(() -> Intake.getInstance().setSetpoint(Intake.IntakeSetpoint.INTAKING))
        // );
        // xboxDriver.getBButton().onTrue(
        //     new InstantCommand(() -> Intake.getInstance().setSetpoint(Intake.IntakeSetpoint.STOWED))
        // );

        // Test snap-to-angle bindings with split superstructure states:
        // xboxDriver.getAButton().onTrue(new InstantCommand(() -> superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.BUMP)));
        // xboxDriver.getAButton().onFalse(new InstantCommand(() -> superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.HOME)));
    }

    private void bindCompetitionDriveHelpers() {
        xboxDriver.getRightTriggerButton(DRIVE_HELPER_TRIGGER_THRESHOLD).whileTrue(
            new RunCommand(this::runHubShotHelper)
        ).onFalse(
            new InstantCommand(() -> superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.TRACKING))
        );

        xboxDriver.getLeftTriggerButton(DRIVE_HELPER_TRIGGER_THRESHOLD).whileTrue(
            new RunCommand(() -> superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.INTAKING))
        ).onFalse(
            new InstantCommand(this::restoreCompetitionDriveIntakeState)
        );

        xboxDriver.getLeftBumper().onTrue(
            new InstantCommand(() -> setCompetitionDriveIntakeReleaseState(
                Superstructure.DesiredIntakeState.ALTERNATING
            ))
        );

        xboxDriver.getRightBumper().whileTrue(
            new RunCommand(this::runClimbAlignmentHelper)
        ).onFalse(
            new InstantCommand(this::releaseClimbAlignmentHelper)
        );

        xboxDriver.getXButton().onTrue(
            new InstantCommand(swerveDrive::toggleTeleopDriveMode)
        );

        xboxDriver.getBButton().whileTrue(
            new RunCommand(this::runAlliancePassHelper)
        ).onFalse(
            new InstantCommand(() -> superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.TRACKING))
        );

        xboxDriver.getYButton().onTrue(
            new InstantCommand(() -> superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.REVERSING))
        ).onFalse(
            new InstantCommand(this::restoreCompetitionDriveIntakeState)
        );
    }

    private void runHubShotHelper() {
        superstructure.setDesiredTargetState(TargetState.HUB);
        superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.SHOOTING);
    }

    private void runAlliancePassHelper() {
        superstructure.getClosestLineOfSightAlliancePassTarget().ifPresentOrElse(
            targetState -> {
                superstructure.setDesiredTargetState(targetState);
                superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.SHOOTING);
            },
            () -> superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.HOME)
        );
    }

    private void runClimbAlignmentHelper() {
        superstructure.setDesiredClimbState(Superstructure.DesiredClimbState.EXTENDED);
        superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.BUMP);
    }

    private void releaseClimbAlignmentHelper() {
        superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.HOME);
        superstructure.setDesiredClimbState(Superstructure.DesiredClimbState.CLIMBED);
    }

    private void setCompetitionDriveIntakeReleaseState(Superstructure.DesiredIntakeState intakeState) {
        competitionDriveIntakeReleaseState = intakeState;
        superstructure.setDesiredIntakeState(intakeState);
    }

    private void restoreCompetitionDriveIntakeState() {
        superstructure.setDesiredIntakeState(competitionDriveIntakeReleaseState);
    }

    private Command followPath(Path path, boolean shouldResetPose) {
        return 
            new InstantCommand(() -> {
                swerveDrive.setCurrentPath(path, shouldResetPose);
            })
            .andThen(setSwerveDriveState(SwerveDrive.DesiredSystemState.FOLLOW_PATH)).
            andThen(new WaitUntilCommand(
                () -> swerveDrive.getCurrentSystemState() == SwerveDrive.CurrentSystemState.IDLE));
    }

    private Command setSwerveDriveState(SwerveDrive.DesiredSystemState state) {
        return new InstantCommand(() -> swerveDrive.setDesiredSystemState(state));
    }

    public void teleopInit() {
        // Ensure we're in teleop state
        competitionDriveIntakeReleaseState = Superstructure.DesiredIntakeState.DEPLOYED;
        swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.TELOP_FIELD_RELATIVE);
        superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.HOME);
        superstructure.setDesiredIntakeState(competitionDriveIntakeReleaseState);
        // dont set climb state leave up to driver
    }

    public void autonomousInit() {
        // Set up for autonomous
        superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.HOME);
        superstructure.setDesiredClimbState(Superstructure.DesiredClimbState.RETRACTED);
        swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.IDLE);
    }

    public void disabledInit() {
        superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.DISABLED);
        superstructure.setDesiredClimbState(Superstructure.DesiredClimbState.DISABLED);
        swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.DISABLED);
    }

    public Command getAutonomousCommand() {
        return followPath(new Path("outpost"), false).andThen(new WaitCommand(8.0)).andThen(followPath(new Path("outpost_to_tower"), false))
        .andThen(new WaitCommand(3.0)).andThen(new InstantCommand(() -> superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.HOME)));
        // return null;
    }
}
