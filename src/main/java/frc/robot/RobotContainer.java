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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.ScoreCommands;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.template.VelocityCommand;
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
    ; // Add a 10% deadband

    /* Setting up bindings for necessary control of the swerve drive platform */

    private static final ProfiledPIDController drivePIDControllerX = new ProfiledPIDController(4.5, 0, .1, new TrapezoidProfile.Constraints(100, 200));
    private static final ProfiledPIDController drivePIDControllerXClose = new ProfiledPIDController(8, 0.05, .15, new TrapezoidProfile.Constraints(100, 200));

    private static final ProfiledPIDController drivePIDControllerY = new ProfiledPIDController(3, 0, .05, new TrapezoidProfile.Constraints(100, 200));
    private static final ProfiledPIDController drivePIDControllerYClose = new ProfiledPIDController(5, 0.0, .25, new TrapezoidProfile.Constraints(100, 200));
    private static final ProfiledPIDController drivePIDControllerYVeryClose = new ProfiledPIDController(7, 0.0, .25, new TrapezoidProfile.Constraints(100, 200));

    public static final PIDController turnPIDController = new PIDController(0.14, 0.0, 0.0);

    public static double xVelocity = 0.03;
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
    private static final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private static final ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();
    private static final WristSubsystem wristSubsystem = WristSubsystem.getInstance();
    private static final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    private static final ClimberSubsystem climberSubsystem = ClimberSubsystem.getInstance();
    public static final AprilTagSubsystem aprilTagSubsystem = AprilTagSubsystem.getInstance();


    public static State state = State.L1;


    // private static final SendableChooser<Command> autoChooser = Autos.getAutoChooser();


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
        NamedCommands.registerCommand("ARML2", ScoreCommands.armL2());
        NamedCommands.registerCommand("ARML3", ScoreCommands.armL3());
        NamedCommands.registerCommand("ARML4", ScoreCommands.armL4());
        NamedCommands.registerCommand("DUNK", ScoreCommands.dunk());
        NamedCommands.registerCommand("ALIGNL", ScoreCommands.autoAlignLAuton());
        NamedCommands.registerCommand("ALIGNR", ScoreCommands.autoAlignRAuton());
        NamedCommands.registerCommand("DRIVE", ScoreCommands.autoMoveForward());

        configureBindings();
        SignalLogger.setPath("/media/LOG/ctre-logs/");
    }

    private void configureBindings() {
        //    commandXboxController.b().onTrue(new InstantCommand(() -> System.out.println("Arm Degrees: " + armSubsystem.getDegrees()))
        //            .andThen(new InstantCommand(() -> System.out.println("Elevator Centimeters: " + elevatorSubsystem.getMechM())))
        //            .andThen(new InstantCommand(() -> System.out.println("Wrist Degrees: " + wristSubsystem.getDegrees()))));

        commandSwerveDrivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                commandSwerveDrivetrain.applyRequest(() -> drive.withVelocityX(-commandXboxController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-commandXboxController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-commandXboxController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                ));
        // reset the field-centric heading on menu button press
        commandXboxController.button(8).onTrue(commandSwerveDrivetrain
                .runOnce(commandSwerveDrivetrain::seedFieldCentric)
                .alongWith(new InstantCommand(() -> commandSwerveDrivetrain.getPigeon2().setYaw(0))));

        if (Utils.isSimulation()) {
            commandSwerveDrivetrain.resetPose(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }

//        commandXboxController.button(9).onTrue(new InstantCommand(() -> algaeControls = false));
//        commandXboxController.button(10).onTrue(new InstantCommand(() -> algaeControls = true));

        commandXboxController.a().onTrue(new InstantCommand(() -> RobotContainer.setState(State.L1)));
        commandXboxController.b().onTrue(ScoreCommands.armL2());
        commandXboxController.x().onTrue(ScoreCommands.armL3());
        commandXboxController.y().onTrue(ScoreCommands.armL4());
//        commandXboxController.b().onTrue(new ConditionalCommand(ScoreCommands.algaeL1(), ScoreCommands.scoreL2(), () -> algaeControls));
//        commandXboxController.x().onTrue(new ConditionalCommand(ScoreCommands.algaeL2(), ScoreCommands.scoreL3(), () -> algaeControls));

        commandXboxController.leftTrigger().onTrue(ScoreCommands.score())
                .onFalse(new VelocityCommand(intakeSubsystem, 0)
                        .alongWith(ScoreCommands.elevatorHP()));
        commandXboxController.rightTrigger().onTrue(new VelocityCommand(intakeSubsystem, 75)
                        .alongWith(ScoreCommands.intakeHP()))
                .onFalse(new VelocityCommand(intakeSubsystem, 0)
                        .alongWith(ScoreCommands.intakeHP()));


        commandXboxController.leftBumper().whileTrue(new SequentialCommandGroup(
                new InstantCommand(() -> commandSwerveDrivetrain.resetRotation(new Rotation2d(
                        Math.toRadians(aprilTagSubsystem.getRotationToAlign(aprilTagSubsystem
                                .getClosestTagID()))))),
                commandSwerveDrivetrain.applyRequest(
                        () -> drive.withVelocityX(xVelocity)
                                .withVelocityY(yVelocity)
                                .withRotationalRate(turnPIDController.calculate(
                                        commandSwerveDrivetrain.getPose().getRotation().getDegrees(), 0))))
        ).onFalse(new InstantCommand(() -> commandSwerveDrivetrain
                .resetRotation(new Rotation2d(Math.toRadians(commandSwerveDrivetrain
                        .getPigeon2().getRotation2d().getDegrees())))));
        commandXboxController.rightBumper().onTrue(new VelocityCommand(intakeSubsystem, -75))
                .onFalse(new VelocityCommand(intakeSubsystem, 0));

        Trigger trigger = commandXboxController.povUp().onTrue(new InstantCommand(() -> climberSubsystem.setPercent(0.6))).onFalse(new InstantCommand(() -> climberSubsystem.setPercent(0)));
        commandXboxController.povDown().onTrue(new InstantCommand(() -> climberSubsystem.setPercent(-0.6))).onFalse(new InstantCommand(() -> climberSubsystem.setPercent(0)));
        //commandXboxController.povDown().onTrue(ScoreCommands.dunk());

        commandXboxController.povRight().onTrue(ScoreCommands.zeroSubsystems());
        // commandXboxController.povLeft().onTrue(ScoreCommands.scoreL1());
        commandXboxController.povLeft().onTrue(new InstantCommand(this::toggleAutoAlignOffset));

        commandSwerveDrivetrain.registerTelemetry(logger::telemeterize);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
//        return Autos.ThreePiece.Blue.threePieceBlueBL4();
        return Autos.TwoPiece.Blue.twoPieceBlueHTopL4();
        //return new PathPlannerAuto("test");
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

//        System.out.println("aligned: " + aligned());
//        System.out.println("X speed: " + commandSwerveDrivetrain.getState().Speeds.vxMetersPerSecond
//                + " Y: " + commandSwerveDrivetrain.getState().Speeds.vyMetersPerSecond);
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
                && Math.abs(aprilTagSubsystem.getClosestTagXYYaw()[1] - autoAlignYOffset) <= .05
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
