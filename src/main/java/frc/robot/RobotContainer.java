// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Autos;
import frc.robot.commands.ScoreCommands;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ElevatorConstants;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.constants.Constants.WristConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.controls.ButtonPanelButtons;
import frc.robot.controls.CommandButtonPanel;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.template.PositionCommand;
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
    private static double MaxAngularRate = 2 * Math.PI; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private static final CommandXboxController commandXboxController = new CommandXboxController(OperatorConstants.driverControllerPort); // My joystick
    private static final CommandXboxController operatorXboxController = new CommandXboxController(OperatorConstants.operatorControllerPort); // My joystick
    private static final CommandButtonPanel commandButtonPanel = new CommandButtonPanel(OperatorConstants.buttonPanel1Port, OperatorConstants.buttonPanel2Port);

    public final static SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDesaturateWheelSpeeds(true)
            .withDeadband(MaxSpeed * .05).withRotationalDeadband(MaxAngularRate * .05) // Add a 10% deadband
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);

    private static final ProfiledPIDController drivePIDControllerX = new ProfiledPIDController(4, 0, .1, new TrapezoidProfile.Constraints(100, 200));
    private static final ProfiledPIDController drivePIDControllerXClose = new ProfiledPIDController(10, 0, .15, new TrapezoidProfile.Constraints(100, 200));

    private static final ProfiledPIDController drivePIDControllerY = new ProfiledPIDController(3, 0, .05, new TrapezoidProfile.Constraints(100, 200));
    private static final ProfiledPIDController drivePIDControllerYClose = new ProfiledPIDController(8, 0.0, .25, new TrapezoidProfile.Constraints(100, 200));
    private static final ProfiledPIDController drivePIDControllerYVeryClose = new ProfiledPIDController(11, 0.0, .25, new TrapezoidProfile.Constraints(100, 200));

    public static final ProfiledPIDController turnPIDController = new ProfiledPIDController(0.175, 0.0, 0.0, new TrapezoidProfile.Constraints(100, 200));

    public static double xVelocity = 0;
    public static double yVelocity = 0;
    public static double rotationVelocity = 0;

    public static double autoAlignXOffset = 0.02;
    public static double autoAlignYOffset = -.165; //.165

    private static TrapezoidProfile profileX = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(1000, 1000));
    private static TrapezoidProfile.State currentStateX = new TrapezoidProfile.State(0, 0);
    private static TrapezoidProfile.State goalStateX = new TrapezoidProfile.State(0, 0);

    private static TrapezoidProfile profileY = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(1000, 1000));
    private static TrapezoidProfile.State currentStateY = new TrapezoidProfile.State(0, 0);
    private static TrapezoidProfile.State goalStateY = new TrapezoidProfile.State(0, 0);

    private static TrapezoidProfile profileRotation = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(1000, 1000));
    private static TrapezoidProfile.State currentStateRotation = new TrapezoidProfile.State(0, 0);
    private static TrapezoidProfile.State goalStateRotation = new TrapezoidProfile.State(0, 0);

    private static Timer timer = new Timer();

    public static final CommandSwerveDrivetrain commandSwerveDrivetrain = TunerConstants.createDrivetrain();
    private static final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private static final ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();
    private static final WristSubsystem wristSubsystem = WristSubsystem.getInstance();
    private static final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    private static final ClimberSubsystem climberSubsystem = ClimberSubsystem.getInstance();
    public static final AprilTagSubsystem aprilTagSubsystem = AprilTagSubsystem.getInstance();

    public static State state = State.L1;
    //    private static final Command auto = Autos.OnePiece.Blue.onePieceBlueHPBL4();
    // private static final SendableChooser<Command> autoChooser = Autos.getAutoChooser();

    // private ObjectDetectionSubsystem objectDetectionSubsystem = ObjectDetectionSubsystem.getInstance();

    private static int selectedReefTag = 0;

    private static List<Integer> reefTags = new ArrayList<>();

    // private Boolean algaeControls = false;

    // The robot's subsystems and commands are defined here...
    private final Telemetry logger = new Telemetry(MaxSpeed);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        commandSwerveDrivetrain.configureAutoBuilder();

//        NamedCommands.registerCommand("INTAKE", ScoreCommands.Intake.intakeAuto());
        NamedCommands.registerCommand("INTAKESEQUENCE", ScoreCommands.Intake.intakeSequence());
//        NamedCommands.registerCommand("OUTTAKE", ScoreCommands.Score.place()
//                .until(() -> !intakeSubsystem.isCoralInIntake()));
//        NamedCommands.registerCommand("DROP", ScoreCommands.Climber.drop());
        NamedCommands.registerCommand("ELEVATORSTABLE", new InstantCommand(() -> elevatorSubsystem.setPosition(0)));
        NamedCommands.registerCommand("HP", ScoreCommands.Intake.intakeHP());
//        NamedCommands.registerCommand("L1", ScoreCommands.Score.scoreL1());
//        NamedCommands.registerCommand("L2", ScoreCommands.Score.scoreL2());
//        NamedCommands.registerCommand("L3", ScoreCommands.Score.scoreL3());
        NamedCommands.registerCommand("L4", ScoreCommands.Score.scoreL4().withTimeout(2));
       // NamedCommands.registerCommand("L4", new PositionCommand(elevatorSubsystem, ElevatorConstants.L4).andThen(new PositionCommand(wristSubsystem, WristConstants.L4)));
//        NamedCommands.registerCommand("ARML2", ScoreCommands.Arm.armL2());
//        NamedCommands.registerCommand("ARML3", ScoreCommands.Arm.armL3());
        NamedCommands.registerCommand("ARML4", ScoreCommands.Arm.armL4());
        NamedCommands.registerCommand("ALIGNL", ScoreCommands.Drive.autoAlignLAuton().withTimeout(2));
        NamedCommands.registerCommand("ALIGNR", ScoreCommands.Drive.autoAlignRAuton().withTimeout(2));
//        NamedCommands.registerCommand("DRIVE", ScoreCommands.Drive.autoMoveForwardBottom());
//        NamedCommands.registerCommand("DRIVETOP", ScoreCommands.Drive.autoMoveForwardTop());

        Autos.initializeAutos();

        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            reefTags.addAll(List.of(7, 8, 9, 10, 11, 6));
        } else {
            reefTags.addAll(List.of(18, 17, 22, 21, 20, 19));
        }

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
        commandXboxController.button(8).onTrue(commandSwerveDrivetrain
                .runOnce(commandSwerveDrivetrain::seedFieldCentric)
                .alongWith(new InstantCommand(() -> commandSwerveDrivetrain.getPigeon2().setYaw(0))));

        if (Utils.isSimulation()) {
            commandSwerveDrivetrain.resetPose(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }

        commandXboxController.a().onTrue(new InstantCommand(() -> RobotContainer.setState(State.L1)));
        commandXboxController.b().onTrue(ScoreCommands.Arm.armL2());
        commandXboxController.x().onTrue(ScoreCommands.Arm.armL3());
        commandXboxController.y().onTrue(ScoreCommands.Arm.armL4());

        commandXboxController.leftTrigger().onTrue(ScoreCommands.Score.score()
                        .alongWith(new ConditionalCommand(
                                ScoreCommands.Drive.autoAlignTeleop(selectedReefTag),
                                ScoreCommands.Drive.autoAlignTeleop(AprilTagSubsystem.getInstance().getClosestTagID()),
                                () -> false)))
                .onFalse(ScoreCommands.Stabling.stable()
                        .alongWith(commandSwerveDrivetrain.applyRequest(() -> drive
                                .withVelocityX(-commandXboxController.getLeftY() * MaxSpeed)
                                .withVelocityY(-commandXboxController.getLeftX() * MaxSpeed)
                                .withRotationalRate(-commandXboxController.getRightX() * MaxAngularRate))));
        commandXboxController.rightTrigger()
                .onTrue(ScoreCommands.Intake.intakeHP().alongWith(ScoreCommands.Intake.intakeSequence()))
                .onFalse(ScoreCommands.Stabling.intakeStable());

        commandXboxController.leftBumper().onTrue(ScoreCommands.Score.score())
                .onFalse(ScoreCommands.Stabling.stable());
        commandXboxController.rightBumper().onTrue(ScoreCommands.Score.place())
                .onFalse(new VelocityCommand(intakeSubsystem, 0));

        commandXboxController.button(7).onTrue(ScoreCommands.Zeroing.zeroSubsystems());
        commandXboxController.povLeft().onTrue(new InstantCommand(this::setAutoAlignOffsetLeft));
        commandXboxController.povRight().onTrue(new InstantCommand(this::setAutoAlignOffsetRight));

        commandXboxController.povUp().onTrue(new InstantCommand(() -> setState(State.ALGAE_HIGH)));
        commandXboxController.povDown().onTrue(new InstantCommand(() -> setState(State.ALGAE_LOW)));

        operatorXboxController.y().onTrue(new InstantCommand(()
                -> armSubsystem.setOffset(armSubsystem.getOffset() + 1)));
        operatorXboxController.x().onTrue(new InstantCommand(()
                -> armSubsystem.setOffset(armSubsystem.getOffset() - 1)));

        operatorXboxController.rightBumper().onTrue(new InstantCommand(()
                -> elevatorSubsystem.setOffset(elevatorSubsystem.getOffset() + .01)));
        operatorXboxController.leftBumper().onTrue(new InstantCommand(()
                -> elevatorSubsystem.setOffset(elevatorSubsystem.getOffset() - .01)));

        operatorXboxController.b().onTrue(new InstantCommand(()
                -> wristSubsystem.setOffset(wristSubsystem.getOffset() + 1)));
        operatorXboxController.a().onTrue(new InstantCommand(()
                -> wristSubsystem.setOffset(wristSubsystem.getOffset() - 1)));

        operatorXboxController.povUp().onTrue(new InstantCommand(() -> climberSubsystem.setPercent(0.6)))
                .onFalse(new InstantCommand(() -> climberSubsystem.setPercent(0)));
        operatorXboxController.povDown().onTrue(new InstantCommand(() -> climberSubsystem.setPercent(-0.6)))
                .onFalse(new InstantCommand(() -> climberSubsystem.setPercent(0)));
        operatorXboxController.povRight().onTrue(new PositionCommand(wristSubsystem, 30));
        operatorXboxController.povLeft().onTrue(new PositionCommand(wristSubsystem, 0));

        operatorXboxController.rightTrigger().onTrue(new PositionCommand(elevatorSubsystem, 0)
                .andThen(new PositionCommand(armSubsystem, 0)));

        commandButtonPanel.button(ButtonPanelButtons.SETPOINT_INTAKE_HP).onTrue(ScoreCommands.Intake.intakeHP().alongWith(ScoreCommands.Intake.intakeSequence()));

        commandButtonPanel.button(ButtonPanelButtons.REEF_SCORE_L1).onTrue(new InstantCommand(() -> RobotContainer.setState(State.L1)));
        commandButtonPanel.button(ButtonPanelButtons.REEF_SCORE_L2).onTrue(ScoreCommands.Arm.armL2());
        commandButtonPanel.button(ButtonPanelButtons.REEF_SCORE_L3).onTrue(ScoreCommands.Arm.armL3());
        commandButtonPanel.button(ButtonPanelButtons.REEF_SCORE_L4).onTrue(ScoreCommands.Arm.armL4());

        commandButtonPanel.button(ButtonPanelButtons.SETMODE_ALGAE).onTrue(new InstantCommand(() -> setState(State.ALGAE_HIGH)));
        commandButtonPanel.button(ButtonPanelButtons.SETMODE_CORAL).onTrue(new InstantCommand(() -> setState(State.ALGAE_LOW)));

        commandButtonPanel.button(ButtonPanelButtons.MOVE_WRIST_INCREASE).whileTrue(new InstantCommand(() -> wristSubsystem.setOffset(wristSubsystem.getOffset() + 1)));
        commandButtonPanel.button(ButtonPanelButtons.MOVE_WRIST_DECREASE).whileTrue(new InstantCommand(() -> wristSubsystem.setOffset(wristSubsystem.getOffset() - 1)));
        commandButtonPanel.button(ButtonPanelButtons.MOVE_ELEVATOR_INCREASE).whileTrue(new InstantCommand(() -> elevatorSubsystem.setOffset(elevatorSubsystem.getOffset() + 0.01)));
        commandButtonPanel.button(ButtonPanelButtons.MOVE_ELEVATOR_DECREASE).whileTrue(new InstantCommand(() -> elevatorSubsystem.setOffset(elevatorSubsystem.getOffset() - 0.01)));
        commandButtonPanel.button(ButtonPanelButtons.MOVE_ARM_INCREASE).whileTrue(new InstantCommand(() -> armSubsystem.setOffset(wristSubsystem.getOffset() + 1)));
        commandButtonPanel.button(ButtonPanelButtons.MOVE_ARM_DECREASE).whileTrue(new InstantCommand(() -> armSubsystem.setOffset(wristSubsystem.getOffset() - 1)));

        commandButtonPanel.button(ButtonPanelButtons.MOVE_CLIMB_INCREASE).onTrue(new InstantCommand(() -> climberSubsystem.setPercent(0.6))).onFalse(new InstantCommand(() -> climberSubsystem.setPercent(0)));
        commandButtonPanel.button(ButtonPanelButtons.MOVE_CLIMB_DECREASE).onTrue(new InstantCommand(() -> climberSubsystem.setPercent(-0.6))).onFalse(new InstantCommand(() -> climberSubsystem.setPercent(0)));

        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_A).onTrue(new InstantCommand(() -> selectedReefTag = reefTags.get(0)).andThen(() -> setAutoAlignOffsetLeft()));
        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_B).onTrue(new InstantCommand(() -> selectedReefTag = reefTags.get(0)).andThen(() -> setAutoAlignOffsetRight()));
        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_C).onTrue(new InstantCommand(() -> selectedReefTag = reefTags.get(1)).andThen(() -> setAutoAlignOffsetLeft()));
        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_D).onTrue(new InstantCommand(() -> selectedReefTag = reefTags.get(1)).andThen(() -> setAutoAlignOffsetRight()));
        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_E).onTrue(new InstantCommand(() -> selectedReefTag = reefTags.get(2)).andThen(() -> setAutoAlignOffsetLeft()));
        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_F).onTrue(new InstantCommand(() -> selectedReefTag = reefTags.get(2)).andThen(() -> setAutoAlignOffsetRight()));
        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_G).onTrue(new InstantCommand(() -> selectedReefTag = reefTags.get(3)).andThen(() -> setAutoAlignOffsetLeft()));
        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_H).onTrue(new InstantCommand(() -> selectedReefTag = reefTags.get(3)).andThen(() -> setAutoAlignOffsetRight()));
        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_I).onTrue(new InstantCommand(() -> selectedReefTag = reefTags.get(4)).andThen(() -> setAutoAlignOffsetLeft()));
        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_J).onTrue(new InstantCommand(() -> selectedReefTag = reefTags.get(4)).andThen(() -> setAutoAlignOffsetRight()));
        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_K).onTrue(new InstantCommand(() -> selectedReefTag = reefTags.get(5)).andThen(() -> setAutoAlignOffsetLeft()));
        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_L).onTrue(new InstantCommand(() -> selectedReefTag = reefTags.get(5)).andThen(() -> setAutoAlignOffsetRight()));

        commandSwerveDrivetrain.registerTelemetry(logger::telemeterize);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return Autos.autonChooserRed.getSelected();
        } else {
            return Autos.autonChooserBlue.getSelected();
        }
    }

    public static void periodic() {
        if (!timer.isRunning()) timer.start();
        if (autoAlignXOffset > 0 && DriverStation.getAlliance().isPresent() &&
                DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red))
            autoAlignXOffset = -autoAlignXOffset;

        currentStateX.position = aprilTagSubsystem.getClosestTagXYYaw()[0];
        currentStateY.position = aprilTagSubsystem.getClosestTagXYYaw()[1];
        currentStateRotation.position = commandSwerveDrivetrain.getPose().getRotation().getDegrees();

        currentStateX.velocity = commandSwerveDrivetrain.getState().Speeds.vxMetersPerSecond;
        currentStateY.velocity = commandSwerveDrivetrain.getState().Speeds.vyMetersPerSecond;
        currentStateRotation.velocity = commandSwerveDrivetrain.getState().Speeds.omegaRadiansPerSecond;

        goalStateX.position = autoAlignXOffset;
        goalStateY.position = autoAlignYOffset;
        goalStateRotation.position = 0;

        TrapezoidProfile.State nextStateX = profileX.calculate(timer.get(), currentStateX, goalStateX);
        TrapezoidProfile.State nextStateY = profileY.calculate(timer.get(), currentStateY, goalStateY);
        TrapezoidProfile.State nextStateRotation = profileRotation.calculate(timer.get(), currentStateRotation, goalStateRotation);

        if ((Math.abs(aprilTagSubsystem.getClosestTagXYYaw()[0] - autoAlignXOffset) > .15
                || Math.abs(aprilTagSubsystem.getClosestTagXYYaw()[1] - autoAlignYOffset) > .1)) {
            xVelocity = drivePIDControllerX.calculate(currentStateX.position, nextStateX);
            yVelocity = drivePIDControllerY.calculate(currentStateY.position, nextStateY);
        } else if (Math.abs(aprilTagSubsystem.getClosestTagXYYaw()[1] - autoAlignYOffset) > .04) {
            xVelocity = drivePIDControllerXClose.calculate(currentStateX.position, nextStateX);
            yVelocity = drivePIDControllerYClose.calculate(currentStateY.position, nextStateY);
        } else {
            xVelocity = drivePIDControllerXClose.calculate(currentStateX.position, nextStateX);
            yVelocity = drivePIDControllerYVeryClose.calculate(currentStateY.position, nextStateY);
        }
        rotationVelocity = turnPIDController.calculate(currentStateRotation.position, nextStateRotation);
//        System.out.println("aligned: " + aligned());
//        System.out.println("X speed: " + commandSwerveDrivetrain.getState().Speeds.vxMetersPerSecond
//                + " Y: " + commandSwerveDrivetrain.getState().Speeds.vyMetersPerSecond);

        System.out.println("Pose: " + commandSwerveDrivetrain.getPose());
        // System.out.println("Drive: " + commandSwerveDrivetrain.getPose().getRotation().getDegrees());
        // System.out.println("Pigeon: " + commandSwerveDrivetrain.getPigeon2().getRotation2d().getDegrees());

//        System.out.println("Elevator: " + elevatorSubsystem.getMechM());
//        System.out.println("Arm: " + armSubsystem.getDegrees());
//        System.out.println("Wrist: " + wristSubsystem.getDegrees());

    }

    public void setAutoAlignOffsetLeft() {
        if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
            if (autoAlignYOffset < 0) {
                autoAlignYOffset = -autoAlignYOffset;
            }
        } else {
            if (autoAlignYOffset > 0) {
                autoAlignYOffset = -autoAlignYOffset;
            }
        }
    }

    public void setAutoAlignOffsetRight() {
        if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
            if (autoAlignYOffset > 0) {
                autoAlignYOffset = -autoAlignYOffset;
            }
        } else {
            if (autoAlignYOffset < 0) {
                autoAlignYOffset = -autoAlignYOffset;
            }
        }
    }

    public void toggleAutoAlignOffset() {
        autoAlignYOffset = -autoAlignYOffset;
    }


    public static boolean aligned() {
        return Math.abs(aprilTagSubsystem.getClosestTagXYYaw()[0] - autoAlignXOffset) <= .02
                && Math.abs(aprilTagSubsystem.getClosestTagXYYaw()[1] - autoAlignYOffset) <= .02;

    }

    public static void setState(State state) {
        RobotContainer.state = state;
    }

    public static State getState() {
        return state;
    }
}
