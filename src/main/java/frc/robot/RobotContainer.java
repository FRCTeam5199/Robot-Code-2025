// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Autos;
import frc.robot.commands.ScoreCommands;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ObjectDetectionSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.utility.State;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private static double MaxSpeed = TunerConstants.kSpeedAt12Volts.baseUnitMagnitude(); // kSpeedAt12VoltsMps desired top speed
    private static double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final CommandXboxController commandXboxController = new CommandXboxController(OperatorConstants.driverControllerPort); // My joystick
    public final static SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDesaturateWheelSpeeds(true)
            .withDeadband(MaxSpeed * .05).withRotationalDeadband(MaxAngularRate * .05)
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    /* Setting up bindings for necessary control of the swerve drive platform */

    private static final ProfiledPIDController drivePIDControllerX = new ProfiledPIDController(3, 0, .1, new TrapezoidProfile.Constraints(100, 200));
    private static final ProfiledPIDController drivePIDControllerXClose = new ProfiledPIDController(6, 0.05, .15, new TrapezoidProfile.Constraints(100, 200));

    private static final ProfiledPIDController drivePIDControllerY = new ProfiledPIDController(2.5, 0, .05, new TrapezoidProfile.Constraints(100, 200));
    private static final ProfiledPIDController drivePIDControllerYClose = new ProfiledPIDController(5, 0.0, .25, new TrapezoidProfile.Constraints(100, 200));
    private static final ProfiledPIDController drivePIDControllerYVeryClose = new ProfiledPIDController(7, 0.0, .25, new TrapezoidProfile.Constraints(100, 200));

    public static final PIDController turnPIDController = new PIDController(0.14, 0.0, 0.0);

    public static double xVelocity = 0;
    public static double yVelocity = 0;

    public static double autoAlignXOffset = 0.0;
    public static double autoAlignYOffset = -.165;

    private static TrapezoidProfile profileX = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(1000, 1000));
    private static TrapezoidProfile.State currentStateX = new TrapezoidProfile.State(0, 0);
    private static TrapezoidProfile.State goalStateX = new TrapezoidProfile.State(0, 0);

    private static TrapezoidProfile profileY = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(1000, 1000));
    private static TrapezoidProfile.State currentStateY = new TrapezoidProfile.State(0, 0);
    private static TrapezoidProfile.State goalStateY = new TrapezoidProfile.State(0, 0);

    private static Timer timer = new Timer();

    public static final CommandSwerveDrivetrain commandSwerveDrivetrain = TunerConstants.createDrivetrain();
    // private static final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    // private static final ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();
    // private static final WristSubsystem wristSubsystem = WristSubsystem.getInstance();
    private static final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    private static final ClimberSubsystem climberSubsystem = ClimberSubsystem.getInstance();
    public static final AprilTagSubsystem aprilTagSubsystem = AprilTagSubsystem.getInstance();

    public static State state = State.L1;

    // private static final SendableChooser<Command> autoChooser = Autos.setupAutoChooser();

    // private ObjectDetectionSubsystem objectDetectionSubsystem = ObjectDetectionSubsystem.getInstance();

    private Boolean algaeControls = false;


    // The robot's subsystems and commands are defined here...
    private final Telemetry logger = new Telemetry(MaxSpeed);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        commandSwerveDrivetrain.configureAutoBuilder();

        NamedCommands.registerCommand("INTAKE", ScoreCommands.intake());
        NamedCommands.registerCommand("OUTTAKE", ScoreCommands.outtake());
        NamedCommands.registerCommand("HP", ScoreCommands.intakeHP());
        NamedCommands.registerCommand("L1", ScoreCommands.scoreL1());
        NamedCommands.registerCommand("L2", ScoreCommands.scoreL2());
        NamedCommands.registerCommand("L3", ScoreCommands.scoreL3());
        NamedCommands.registerCommand("L4", ScoreCommands.scoreL4());
        NamedCommands.registerCommand("armL2", ScoreCommands.armL2());
        NamedCommands.registerCommand("armL3", ScoreCommands.armL3());
        NamedCommands.registerCommand("armL4", ScoreCommands.armL4());
        NamedCommands.registerCommand("dunk", ScoreCommands.dunk());
        NamedCommands.registerCommand("alignL", ScoreCommands.autoAlignLAuton());
        NamedCommands.registerCommand("alignR", ScoreCommands.autoAlignRAuton());
        NamedCommands.registerCommand("moveIntake", ScoreCommands.autoMoveForward());

        configureBindings();
        SignalLogger.setPath("/media/LOG/ctre-logs/");
    }

    private void configureBindings() {
        commandSwerveDrivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                commandSwerveDrivetrain.applyRequest(() -> drive.withVelocityX(-commandXboxController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-commandXboxController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-commandXboxController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                ));
        // reset the field-centric heading on menu button press
        // Window button
        commandXboxController.button(7).onTrue(commandSwerveDrivetrain
                .runOnce(commandSwerveDrivetrain::seedFieldCentric)
                .alongWith(new InstantCommand(() -> commandSwerveDrivetrain.getPigeon2().setYaw(0))));

        commandXboxController.button(8).toggleOnTrue(commandSwerveDrivetrain.applyRequest(() -> brake));

        commandXboxController.a().onTrue(ScoreCommands.stable());
        commandXboxController.x().onTrue(ScoreCommands.scoreL2());
        commandXboxController.b().onTrue(ScoreCommands.scoreL3());
        commandXboxController.y().onTrue(ScoreCommands.scoreL4());

        commandXboxController.leftBumper().onTrue(ScoreCommands.intakeHP());

        commandXboxController.rightBumper().onTrue(ScoreCommands.dunk());

        commandXboxController.leftTrigger().onTrue(new InstantCommand(() -> intakeSubsystem.setPercent(1)))
                .onFalse(new InstantCommand(() -> intakeSubsystem.setPercent(0)));
        commandXboxController.rightTrigger().onTrue(new InstantCommand(() -> intakeSubsystem.setPercent(-1)))
                .onFalse(new InstantCommand(() -> intakeSubsystem.setPercent(0)));

        commandXboxController.povUp().onTrue(new InstantCommand(() -> climberSubsystem.setPercent(0.3))).onFalse(new InstantCommand(() -> climberSubsystem.setPercent(0)));
        commandXboxController.povDown().onTrue(new InstantCommand(() -> climberSubsystem.setPercent(-0.3))).onFalse(new InstantCommand(() -> climberSubsystem.setPercent(0)));

        commandXboxController.povLeft().onTrue(new InstantCommand(this::toggleAutoAlignOffset));
        commandXboxController.povRight().onTrue(ScoreCommands.scoreL1());


        if (Utils.isSimulation()) {
            commandSwerveDrivetrain.resetPose(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }

        commandSwerveDrivetrain.registerTelemetry(logger::telemeterize);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // return autoChooser.getSelected();
        return null;
    }

    public static void periodic() {
        if (!timer.isRunning()) timer.start();

        currentStateX.position = aprilTagSubsystem.getClosestTagXYYaw()[0];
        currentStateY.position = aprilTagSubsystem.getClosestTagXYYaw()[1];
        currentStateX.velocity = commandSwerveDrivetrain.getState().Speeds.vxMetersPerSecond;
        currentStateY.velocity = commandSwerveDrivetrain.getState().Speeds.vyMetersPerSecond;

        goalStateX.position = autoAlignXOffset;
        goalStateY.position = autoAlignYOffset;

        TrapezoidProfile.State nextStateX = profileX.calculate(timer.get(), currentStateX, goalStateX);
        TrapezoidProfile.State nextStateY = profileY.calculate(timer.get(), currentStateY, goalStateY);

        if ((Math.abs(aprilTagSubsystem.getClosestTagXYYaw()[0] - autoAlignXOffset) > .15
                || Math.abs(aprilTagSubsystem.getClosestTagXYYaw()[1] - autoAlignYOffset) > .075)) {
            xVelocity = drivePIDControllerX.calculate(currentStateX.position, nextStateX);
            yVelocity = drivePIDControllerY.calculate(currentStateY.position, nextStateY);
        } else if (Math.abs(aprilTagSubsystem.getClosestTagXYYaw()[1] - autoAlignYOffset) > .04) {
            xVelocity = drivePIDControllerXClose.calculate(currentStateX.position, nextStateX);
            yVelocity = drivePIDControllerYClose.calculate(currentStateY.position, nextStateY);
        } else {
            xVelocity = drivePIDControllerXClose.calculate(currentStateX.position, nextStateX);
            yVelocity = drivePIDControllerYVeryClose.calculate(currentStateY.position, nextStateY);
        }

        if (UserInterface.getControlComponent("Reset All").getBoolean(false)) {
            CommandScheduler.getInstance().schedule(ScoreCommands.zeroSubsystems());
            UserInterface.setControlComponent("Reset All", false);
        }
        if (UserInterface.getControlComponent("Reset Elevator").getBoolean(false)) {
            CommandScheduler.getInstance().schedule(ScoreCommands.zeroElevator());
            UserInterface.setControlComponent("Reset Elevator", false);
        }
        if (UserInterface.getControlComponent("Reset Arm").getBoolean(false)) {
            CommandScheduler.getInstance().schedule(ScoreCommands.zeroArm());
            UserInterface.setControlComponent("Reset Arm", false);
        }
        if (UserInterface.getControlComponent("Reset Wrist").getBoolean(false)) {
            CommandScheduler.getInstance().schedule(ScoreCommands.zeroWrist());
            UserInterface.setControlComponent("Reset Wrist", false);
        }

        UserInterface.setAutonComponent("Event", DriverStation.getEventName());
        UserInterface.setAutonComponent("Game Message", DriverStation.getGameSpecificMessage());
        UserInterface.setAutonComponent("Location", DriverStation.getLocation().getAsInt());
        UserInterface.setAutonComponent("Alliance", DriverStation.getAlliance().get() == Alliance.Blue);
        UserInterface.setAutonComponent("Enabled", DriverStation.isEnabled());
        UserInterface.setAutonComponent("EStop", DriverStation.isEStopped());
        UserInterface.setAutonComponent("Match Type", DriverStation.getMatchType().toString());
        UserInterface.setAutonComponent("Match Number", DriverStation.getMatchNumber());
        UserInterface.setAutonComponent("Replay Match Number", DriverStation.getReplayNumber());
        UserInterface.setAutonComponent("Match Time", DriverStation.getMatchTime());

        UserInterface.setTeleopComponent("Event", DriverStation.getEventName());
        UserInterface.setTeleopComponent("Game Message", DriverStation.getGameSpecificMessage());
        UserInterface.setTeleopComponent("Location", DriverStation.getLocation().getAsInt());
        UserInterface.setTeleopComponent("Enabled", DriverStation.isEnabled());
        UserInterface.setTeleopComponent("EStop", DriverStation.isEStopped());
        UserInterface.setTeleopComponent("Alliance", DriverStation.getAlliance().get() == Alliance.Blue);
        UserInterface.setTeleopComponent("Match Type", DriverStation.getMatchType().toString());
        UserInterface.setTeleopComponent("Match Number", DriverStation.getMatchNumber());
        UserInterface.setTeleopComponent("Replay Match Number", DriverStation.getReplayNumber());
        UserInterface.setTeleopComponent("Match Time", DriverStation.getMatchTime());
    }


    public void toggleAutoAlignOffsetLeft() {
        if (autoAlignYOffset > 0) {
            autoAlignYOffset = -autoAlignYOffset;
        }
    }

    public void toggleAutoAlignOffsetRight() {
        if (autoAlignYOffset < 0) {
            autoAlignYOffset = -autoAlignYOffset;
        }
    }

    public void toggleAutoAlignOffset() {
        autoAlignYOffset = -autoAlignYOffset;
    }


    public static boolean aligned() {
        return Math.abs(aprilTagSubsystem.getClosestTagXYYaw()[0] - autoAlignXOffset) <= .05
                && Math.abs(aprilTagSubsystem.getClosestTagXYYaw()[1] - autoAlignYOffset) <= .04
                && Math.abs(commandSwerveDrivetrain.getState().Speeds.vxMetersPerSecond) <= .01
                && Math.abs(commandSwerveDrivetrain.getState().Speeds.vyMetersPerSecond) <= .01;

    }

    public static void setState(State state) {
        RobotContainer.state = state;
    }

    public static State getState() {
        return state;
    }
}
