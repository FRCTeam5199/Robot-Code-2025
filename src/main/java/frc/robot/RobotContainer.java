// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ScoreCommands;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.template.VelocityCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.baseUnitMagnitude(); // kSpeedAt12VoltsMps desired top speed
    private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity


    /* Setting up bindings for necessary control of the swerve drive platform */
    private final CommandXboxController commandXboxController = new CommandXboxController(OperatorConstants.driverControllerPort); // My joystick
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDesaturateWheelSpeeds(true)
            .withDeadband(MaxSpeed * .05).withRotationalDeadband(MaxAngularRate * .05) // Add a 10% deadband
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage); // I want field-centric


    private static final ProfiledPIDController drivePIDControllerX = new ProfiledPIDController(2, 0.0, .1, new TrapezoidProfile.Constraints(100, 200));
    private static final ProfiledPIDController drivePIDControllerXClose = new ProfiledPIDController(5.25, 0, .15, new TrapezoidProfile.Constraints(100, 200));

    private static final ProfiledPIDController drivePIDControllerY = new ProfiledPIDController(2.25, 0.0, .05, new TrapezoidProfile.Constraints(100, 200));
    private static final ProfiledPIDController drivePIDControllerYClose = new ProfiledPIDController(5, 0, .15, new TrapezoidProfile.Constraints(100, 200));

    private static final PIDController turnPIDController = new PIDController(0.125, 0.0, 0.0);

    private static double xVelocity = 0;
    private static double yVelocity = 0;

    private static double autoAlignXOffset = -.02;
    private static double autoAlignYOffset = -.17;

    private static TrapezoidProfile profileX = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(1000, 1000));
    private static TrapezoidProfile.State currentStateX = new TrapezoidProfile.State(0, 0);
    private static TrapezoidProfile.State goalStateX = new TrapezoidProfile.State(0, 0);

    private static TrapezoidProfile profileY = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(1000, 1000));
    private static TrapezoidProfile.State currentStateY = new TrapezoidProfile.State(0, 0);
    private static TrapezoidProfile.State goalStateY = new TrapezoidProfile.State(0, 0);

    private static Timer timer = new Timer();

    public static final CommandSwerveDrivetrain commandSwerveDrivetrain = TunerConstants.createDrivetrain(); // My commandSwerveDrivetrain
    private static final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private static final ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();
    private static final WristSubsystem wristSubsystem = WristSubsystem.getInstance();
    private static final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    private static final ClimberSubsystem climberSubsystem = ClimberSubsystem.getInstance();
    private static final AprilTagSubsystem aprilTagSubsystem = AprilTagSubsystem.getInstance();

    // private static final SendableChooser<Command> autoChooser = Autos.getAutoChooser();


    private ObjectDetectionSubsystem objectDetectionSubsystem = ObjectDetectionSubsystem.getInstance();

    private Boolean algaeControls = false;


    // The robot's subsystems and commands are defined here...
    private final Telemetry logger = new Telemetry(MaxSpeed);
    //    public static final ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();
//    public static final RollerTestSubsystem rollerTestSubsystem = RollerTestSubsystem.getInstance();
//    public static final LinearTestSubsystem linearTestSubsystem = new LinearTestSubsystem();
    //  public static final PivotTestSubsystem pivotTestSubsystem = new PivotTestSubsystem();
    // private final SendableChooser<Command> autoChooser = Autos.getAutoChooser();

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
        configureBindings();
        SignalLogger.setPath("/media/LOG/ctre-logs/");
    }

    private void configureBindings() {
        commandSwerveDrivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                commandSwerveDrivetrain.applyRequest(() -> drive.withVelocityX(-commandXboxController.getLeftY() * MaxSpeed))); // Drive forward with negative Y (forward)

//        commandXboxController.leftBumper().onTrue(Commands.runOnce(SignalLogger::start).alongWith(new PrintCommand("Start")));
//        commandXboxController.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop).alongWith(new PrintCommand("End")));
//
//        commandXboxController.povLeft().onTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
//        commandXboxController.povRight().onTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
//        commandXboxController.povUp().onTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
//        commandXboxController.povDown().onTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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

//        commandXboxController.rightBumper().onTrue(new InstantCommand(() -> System.out.println("Arm Degrees: " + armSubsystem.getDegrees()))
//                .andThen(new InstantCommand(() -> System.out.println("Elevator Meters: " + elevatorSubsystem.getMechM())))
//                .andThen(new InstantCommand(() -> System.out.println("Wrist Degrees: " + wrist.getDegrees()))));

        commandXboxController.rightBumper().whileTrue(new SequentialCommandGroup(
                new InstantCommand(() -> commandSwerveDrivetrain.resetRotation(new Rotation2d(
                        Math.toRadians(aprilTagSubsystem.getRotationToAlign(aprilTagSubsystem
                                .getClosestTagID()))))),
                commandSwerveDrivetrain.applyRequest(
                        () -> drive.withVelocityX(xVelocity)
                                .withVelocityY(yVelocity)
                                .withRotationalRate(turnPIDController.calculate(
                                        commandSwerveDrivetrain.getPose().getRotation().getDegrees(), 0))))
                .alongWith(new InstantCommand(() -> intakeSubsystem.setPercent(.1)))
        ).onFalse(new InstantCommand(() -> commandSwerveDrivetrain
                .resetRotation(new Rotation2d(Math.toRadians(commandSwerveDrivetrain
                        .getPigeon2().getRotation2d().getDegrees()))))
                .alongWith(new InstantCommand(() -> intakeSubsystem.setPercent(0))));
//        commandXboxController.rightBumper().onTrue(new InstantCommand(()
//                -> commandSwerveDrivetrain.resetRotation(new Rotation2d(Math.toRadians(-180 - 240 + commandSwerveDrivetrain.getPose().getRotation().getDegrees())))));


        commandXboxController.button(9).onTrue(new InstantCommand(() -> algaeControls = false));
        commandXboxController.button(10).onTrue(new InstantCommand(() -> algaeControls = true));

        commandXboxController.a().onTrue(ScoreCommands.intakeHP());
        commandXboxController.b().onTrue(new ConditionalCommand(ScoreCommands.algaeL1(), ScoreCommands.scoreL2(), () -> algaeControls));
        commandXboxController.x().onTrue(new ConditionalCommand(ScoreCommands.algaeL2(), ScoreCommands.scoreL3(), () -> algaeControls));
        commandXboxController.y().onTrue(ScoreCommands.scoreL4());

        /*
        arm - , elevator - , wrist - hp
        arm - , elevator - , wrist - l1
        arm - , elevator - , wrist - l2
        arm - , elevator - , wrist - l3
        arm - , elevator - , wrist - l4
         */

        commandXboxController.leftBumper().onTrue(new ConditionalCommand(ScoreCommands.algaeStable(), ScoreCommands.stable(), () -> algaeControls));

        commandXboxController.leftTrigger().onTrue(new InstantCommand(() -> intakeSubsystem.setPercent(-1))) //Outtake
                .onFalse(new InstantCommand(() -> intakeSubsystem.setPercent(0)));
        commandXboxController.rightTrigger().onTrue(new InstantCommand(() -> intakeSubsystem.setPercent(1))) //Intake
                .onFalse(new InstantCommand(() -> intakeSubsystem.setPercent(0))
                        // Lock on with limelight thing, if breaking comment the entire andThen statement
//                    .andThen(new InstantCommand(() -> drivetrain.applyRequest(() -> drive
//                            .withVelocityX(0.0)
//                            .withVelocityY(0.0)
//                            .withRotationalRate(objectDetectionSubsystem.lockOn()))))
                );

        commandXboxController.povUp().onTrue(new InstantCommand(() -> climberSubsystem.setPercent(0.3))).onFalse(new InstantCommand(() -> climberSubsystem.setPercent(0)));
        commandXboxController.povDown().onTrue(new InstantCommand(() -> climberSubsystem.setPercent(-0.3))).onFalse(new InstantCommand(() -> climberSubsystem.setPercent(0)));

        commandXboxController.povRight().onTrue(ScoreCommands.zeroElevator())
                .onFalse(new VelocityCommand(elevatorSubsystem, 0));
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
        // return Autos.exampleAuto(armSubsystem);
        // return autoChooser.getSelected();
        return new PathPlannerAuto("1 Piece Blue Bottom L1");
    }

    // public static Command threePieceProcessor() {
    //     return AutoBuilder.buildAuto("Lfue");
    // }

    public static void periodic() {
        if (!timer.isRunning()) timer.start();

        if ((Math.abs(aprilTagSubsystem.getClosestTagXYYaw()[0] - autoAlignXOffset) > .15
                || Math.abs(aprilTagSubsystem.getClosestTagXYYaw()[1]) - Math.abs(autoAlignYOffset) > .15)) {
            currentStateX.position = aprilTagSubsystem.getClosestTagXYYaw()[0];
            currentStateY.position = aprilTagSubsystem.getClosestTagXYYaw()[1];

            goalStateX.position = autoAlignXOffset;
            goalStateY.position = autoAlignYOffset;

            TrapezoidProfile.State nextStateX = profileX.calculate(timer.get(), currentStateX, goalStateX);
            TrapezoidProfile.State nextStateY = profileY.calculate(timer.get(), currentStateY, goalStateY);

            xVelocity = drivePIDControllerX.calculate(currentStateX.position, nextStateX);
            yVelocity = drivePIDControllerY.calculate(currentStateY.position, nextStateY);
        } else {
            xVelocity = drivePIDControllerXClose.calculate(aprilTagSubsystem.getClosestTagXYYaw()[0], autoAlignXOffset);
            yVelocity = drivePIDControllerYClose.calculate(aprilTagSubsystem.getClosestTagXYYaw()[1], autoAlignYOffset);
        }
    }

    public void toggleAutoAlignOffset() {
        autoAlignYOffset = -autoAlignYOffset;
    }
}
