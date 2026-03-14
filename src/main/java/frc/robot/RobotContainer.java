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

        // Configure teleop input suppliers for SwerveDrive FSM
        // Using normalized inputs (-1 to 1) with deadband applied
        swerveDrive.setTeleopInputSuppliers(
            () -> -MathUtil.applyDeadband(xboxDriver.getLeftY(), Constants.OperatorConstants.LEFT_Y_DEADBAND),
            () -> -MathUtil.applyDeadband(xboxDriver.getLeftX(), Constants.OperatorConstants.LEFT_X_DEADBAND),
            () -> -MathUtil.applyDeadband(xboxDriver.getRightX(), Constants.OperatorConstants.RIGHT_X_DEADBAND)
        );

        // Set default state to TELEOP
        swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.TELEOP);
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

        Trigger getRightBumper = new Trigger(() -> xboxDriver.getRightTrigger() > 0.5);
        getRightBumper.onTrue(
            new InstantCommand(() -> superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.SHOOTING))
        );
        getRightBumper.onFalse(
            new InstantCommand(() -> superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.HOME))
        );

        Trigger getLeftBumper = new Trigger(() -> xboxDriver.getLeftTrigger() > 0.5);
        getLeftBumper.onTrue(
            new InstantCommand(() -> superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.INTAKING))
        );
        getLeftBumper.onFalse(
            new InstantCommand(() -> superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.DEPLOYED))
        );


        xboxDriver.getBButton().onTrue(
            new InstantCommand(() -> superstructure.setDesiredTargetState(Superstructure.TargetState.PASS_ALLIANCE_BOTTOM))
        );
        xboxDriver.getBButton().onFalse(
            new InstantCommand(() -> superstructure.setDesiredTargetState(Superstructure.TargetState.HUB))
        );
        xboxDriver.getXButton().onTrue(
            new InstantCommand(() -> superstructure.setDesiredTargetState(Superstructure.TargetState.PASS_ALLIANCE_TOP))
        );
        xboxDriver.getXButton().onFalse(
            new InstantCommand(() -> superstructure.setDesiredTargetState(Superstructure.TargetState.HUB))
        );        

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
        swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.TELEOP);
        superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.HOME);
        superstructure.setDesiredClimbState(Superstructure.DesiredClimbState.RETRACTED);
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
