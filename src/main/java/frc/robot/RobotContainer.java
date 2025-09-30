// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Autos;
import frc.robot.commands.ScoreCommands;
import frc.robot.commands.ScoreCommands.Score;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ElevatorConstants;
import frc.robot.constants.Constants.OperatorConstants;
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
import frc.robot.utility.ScoringPosition;
import frc.robot.utility.State;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.baseUnitMagnitude(); // kSpeedAt12VoltsMps desired top speed
    public static double MaxAngularRate = 2 * Math.PI; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    public static final CommandXboxController commandXboxController = new CommandXboxController(OperatorConstants.driverControllerPort); // My joystick
    private static final CommandXboxController operatorXboxController = new CommandXboxController(OperatorConstants.operatorControllerPort); // My joystick
    private static final CommandButtonPanel commandButtonPanel = new CommandButtonPanel(OperatorConstants.buttonPanel1Port, OperatorConstants.buttonPanel2Port);

    public final static SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDesaturateWheelSpeeds(true)
            .withDeadband(MaxSpeed * .05).withRotationalDeadband(MaxAngularRate * .05) // Add a 10% deadband
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);

    // Robot Centric drive to use with auto-balancing
    public final static SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric().withDesaturateWheelSpeeds(true)
            .withDeadband(MaxSpeed * .05).withRotationalDeadband(MaxAngularRate * .05) // Add a 10% deadband
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);

    private static final ProfiledPIDController drivePIDControllerX = new ProfiledPIDController(4, 0, .1, new TrapezoidProfile.Constraints(100, 200));
    private static final ProfiledPIDController drivePIDControllerXClose = new ProfiledPIDController(10, 0, .15, new TrapezoidProfile.Constraints(100, 200));

    private static final ProfiledPIDController drivePIDControllerY = new ProfiledPIDController(3, 0, .05, new TrapezoidProfile.Constraints(100, 200));
    private static final ProfiledPIDController drivePIDControllerYClose = new ProfiledPIDController(8, 0.0, .25, new TrapezoidProfile.Constraints(100, 200));
    private static final ProfiledPIDController drivePIDControllerYVeryClose = new ProfiledPIDController(11, 0.0, .25, new TrapezoidProfile.Constraints(100, 200));

    public static final ProfiledPIDController turnPIDController = new ProfiledPIDController(0.175, 0.0, 0.0, new TrapezoidProfile.Constraints(100, 200));

    public static final PIDController turnToPiecePIdController = new PIDController(.05, 0.0, 0.0);
    public static final PIDController turnToPiecePIdControllerClose = new PIDController(.1, 0.0, 0.0);
    public static final PIDController turnToPiecePIdControllerVeryClose = new PIDController(.15, 0.0, 0.0);

    public static double xVelocity = 0;
    public static double yVelocity = 0;
    public static double rotationVelocity = 0;
    public static double driveToPieceRotationVelocity;

    public static double autoAlignXOffset = 0.04;
    public static double autoAlignYOffset = -.15;

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

    private static TrapezoidProfile driveToPieceRotation = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(1000, 1000));
    private static TrapezoidProfile.State driveToPieceCurrentState = new TrapezoidProfile.State(0, 0);
    private static TrapezoidProfile.State driveToPieceGoalState = new TrapezoidProfile.State(0, 0);

    public static final CommandSwerveDrivetrain commandSwerveDrivetrain = TunerConstants.createDrivetrain();
    private static final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private static final ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();
    private static final WristSubsystem wristSubsystem = WristSubsystem.getInstance();
    private static final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    private static final ClimberSubsystem climberSubsystem = ClimberSubsystem.getInstance();

    public static final AprilTagSubsystem aprilTagSubsystem = AprilTagSubsystem.getInstance();

    // private ObjectDetectionSubsystem objectDetectionSubsystem = ObjectDetectionSubsystem.getInstance();

    // private static final SendableChooser<Command> autoChooser = Autos.getAutoChooser();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private static int selectedReefTag = 0;
    private static boolean lockOnMode = false;
    public static State state = State.L4;
    private static Timer timer = new Timer();
    private static boolean useAutoAlign = true;
    private static boolean shouldFixTip = true;
    private static boolean isCoralBlockingHP = false;

    public static ScoringPosition getCurrentScoringPosition() {
        return currentScoringPosition;
    }

    private static ScoringPosition currentScoringPosition = ScoringPosition.REEF_SIDE_A;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        NamedCommands.registerCommand("ARML4", ScoreCommands.Arm.armL4());
        NamedCommands.registerCommand("PREPL4", ScoreCommands.Score.prepL4());
        NamedCommands.registerCommand("UNWIND", ScoreCommands.Climber.slightUnwindAuton()
                .andThen(ScoreCommands.Arm.armL4()));
        NamedCommands.registerCommand("UNWINDL4", ScoreCommands.Climber.slightUnwindAuton()
                .andThen(ScoreCommands.Arm.armL4())
                .andThen(ScoreCommands.Score.scoreL4()));
        NamedCommands.registerCommand("L4", Score.scoreL4().withTimeout(3));
        NamedCommands.registerCommand("HP", ScoreCommands.Intake.intakeHP());


        NamedCommands.registerCommand("ALIGNL", ScoreCommands.Drive.autoAlignLAuton()
                .withTimeout(2.5));
        NamedCommands.registerCommand("ALIGNR", ScoreCommands.Drive.autoAlignRAuton()
                .withTimeout(2.5));
        NamedCommands.registerCommand("DRIVE", ScoreCommands.Drive.autoMoveForwardBottom()
                .withTimeout(3));
        NamedCommands.registerCommand("DRIVETOP", ScoreCommands.Drive.autoMoveForwardTop()
                .withTimeout(3));

        NamedCommands.registerCommand("INTAKE", new VelocityCommand(intakeSubsystem, 40, 40)
                .until(intakeSubsystem::hasCoral));
        NamedCommands.registerCommand("OUTTAKE", ScoreCommands.Score.place()
                .until(() -> intakeSubsystem.isAboveSpeed() && !intakeSubsystem.hasCoral())
                .withTimeout(2)
                .andThen(new InstantCommand(() -> wristSubsystem.setPosition(Constants.WristConstants.HP))));

        NamedCommands.registerCommand("DROP", ScoreCommands.Climber.drop());

        Autos.initializeAutos();

        configureBindings();
        SignalLogger.setPath("/media/BRAZIL/");
        SignalLogger.writeBoolean("Camera Present", aprilTagSubsystem.camera.isConnected());
        SignalLogger.writeDouble("X Value", aprilTagSubsystem.getClosestTagXYYaw()[0]);
        SignalLogger.writeDouble("Y Value", aprilTagSubsystem.getClosestTagXYYaw()[1]);

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

        commandXboxController.a().onTrue(ScoreCommands.Arm.armL1());
        commandXboxController.b().onTrue(ScoreCommands.Arm.armL2());
        commandXboxController.x().onTrue(ScoreCommands.Arm.armL3());
        commandXboxController.y().onTrue(ScoreCommands.Arm.armL4());

        commandXboxController.leftTrigger().onTrue(Autos.autoScore())
                .onFalse(ScoreCommands.Stabling.stable()
                        .alongWith(commandSwerveDrivetrain.applyRequest(() -> drive
                                .withVelocityX(-commandXboxController.getLeftY() * MaxSpeed)
                                .withVelocityY(-commandXboxController.getLeftX() * MaxSpeed)
                                .withRotationalRate(-commandXboxController.getRightX() * MaxAngularRate))));


        commandXboxController.rightTrigger().onTrue(ScoreCommands.Score.score()
                        .alongWith(ScoreCommands.Drive.autoAlignTeleop())
                        .alongWith(new VelocityCommand(intakeSubsystem, 60, 60)
                                .onlyIf(() -> state == State.BARGE
                                        || state == State.PROCESSOR
                                        || state == State.ALGAE_LOW
                                        || state == State.ALGAE_HIGH)))
                .onFalse(ScoreCommands.Stabling.stable()
                        .alongWith(new VelocityCommand(intakeSubsystem, 60, 60)
                                .onlyIf(() -> state == State.ALGAE_LOW || state == State.ALGAE_HIGH))
                        .alongWith(commandSwerveDrivetrain.applyRequest(() -> drive
                                .withVelocityX(-commandXboxController.getLeftY() * MaxSpeed)
                                .withVelocityY(-commandXboxController.getLeftX() * MaxSpeed)
                                .withRotationalRate(-commandXboxController.getRightX() * MaxAngularRate))));

        commandXboxController.leftBumper().onTrue(
                        new ConditionalCommand(
                                ScoreCommands.Intake.intakeGroundAlgae(),
                                ScoreCommands.Intake.intakeGround()
                                        .until(intakeSubsystem::hasCoral)
                                        .andThen(ScoreCommands.Stabling.groundIntakeStable()),
                                () -> state == State.BARGE || state == State.PROCESSOR
                        ))
                .onFalse(new ConditionalCommand(
                        ScoreCommands.Arm.armStable()
                                .alongWith(new VelocityCommand(intakeSubsystem, 60, 60)),
                        ScoreCommands.Stabling.groundIntakeStable(),
                        () -> state == State.BARGE || state == State.PROCESSOR
                ));

        commandXboxController.rightBumper().onTrue(ScoreCommands.Score.place())
                .onFalse(
                        new ConditionalCommand(
                                new VelocityCommand(intakeSubsystem, 30, 30),
                                new VelocityCommand(intakeSubsystem, 0, 0),
                                () -> RobotContainer.getState() == State.BARGE
                                        || RobotContainer.getState() == State.PROCESSOR
                        ));
        commandXboxController.button(7).onTrue(ScoreCommands.Zeroing.zeroSubsystems());

        commandXboxController.povLeft().onTrue(new InstantCommand(RobotContainer::setAutoAlignOffsetLeft));
        commandXboxController.povRight().onTrue(new InstantCommand(RobotContainer::setAutoAlignOffsetRight));
//        commandXboxController.povLeft().onTrue(new VelocityCommand(intakeSubsystem, -100, -100))
//                .onFalse(new VelocityCommand(intakeSubsystem, 0, 0));
//        commandXboxController.povRight().onTrue(new VelocityCommand(intakeSubsystem, 100, 100))
//                .onFalse(new VelocityCommand(intakeSubsystem, 0, 0));


//        commandXboxController.povUp().onTrue(ScoreCommands.Arm.armAlgaeHigh());
//        commandXboxController.povDown().onTrue(ScoreCommands.Arm.armAlgaeLow());
        commandXboxController.povUp().onTrue(ScoreCommands.Drive.driveToPiece())
                .onFalse(commandSwerveDrivetrain.applyRequest(() -> drive
                        .withVelocityX(-commandXboxController.getLeftY() * MaxSpeed)
                        .withVelocityY(-commandXboxController.getLeftX() * MaxSpeed)
                        .withRotationalRate(-commandXboxController.getRightX() * MaxAngularRate)));
//        commandXboxController.povUp().onTrue(ScoreCommands.Drive.driveToPiece().alongWith(
//                        ScoreCommands.Intake.intakeGround()
//                                .until(intakeSubsystem::hasCoral)
//                                .andThen(ScoreCommands.Stabling.groundIntakeStable())
//                ))
//                .onFalse(commandSwerveDrivetrain.applyRequest(() -> drive
//                                .withVelocityX(-commandXboxController.getLeftY() * MaxSpeed)
//                                .withVelocityY(-commandXboxController.getLeftX() * MaxSpeed)
//                                .withRotationalRate(-commandXboxController.getRightX() * MaxAngularRate))
//                        .alongWith(ScoreCommands.Stabling.groundIntakeStable()
//                        ));

        // commandXboxController.povUp().onTrue(new InstantCommand(() -> climberSubsystem.setPercent(1)))
        //         .onFalse(new InstantCommand(() -> climberSubsystem.setPercent(0)));
        // commandXboxController.povDown().onTrue(new InstantCommand(() -> climberSubsystem.setPercent(-1)))
        //         .onFalse(new InstantCommand(() -> climberSubsystem.setPercent(0)));

//        commandButtonPanel.button(ButtonPanelButtons.SETPOINT_INTAKE_HP).onTrue(ScoreCommands.Intake.intakeHP())
//                .onFalse(ScoreCommands.Stabling.intakeStable()
//                        .alongWith(new ConditionalCommand(
//                                ScoreCommands.Intake.intakeSequence(),
//                                new VelocityCommand(intakeSubsystem, 0, 0),
//                                intakeSubsystem::hasCoral
//                        )));
        //Intake when Coral is in front of HP
//        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_L).
//                onTrue(new InstantCommand(RobotContainer::toggleCoralBlockingHP));

        //Outtake
        commandButtonPanel.button(ButtonPanelButtons.SETPOINT_INTAKE_HP)
                .onTrue(new VelocityCommand(intakeSubsystem, -50, -50))
                .onFalse(new VelocityCommand(intakeSubsystem, 0, 0));

        commandButtonPanel.button(ButtonPanelButtons.REEF_SCORE_L2).onTrue(ScoreCommands.Arm.armProcessor()
                .alongWith(new VelocityCommand(intakeSubsystem, 60, 60)));
        commandButtonPanel.button(ButtonPanelButtons.REEF_SCORE_L4).onTrue(ScoreCommands.Arm.armBarge()
                .alongWith(new VelocityCommand(intakeSubsystem, 60, 60)));

        commandXboxController.povDown()
                .onTrue(ScoreCommands.Arm.armAlgaeHigh());
        commandButtonPanel.button(ButtonPanelButtons.SETMODE_CORAL)
                .onTrue(ScoreCommands.Arm.armAlgaeLow());

        commandButtonPanel.button(ButtonPanelButtons.MOVE_WRIST_INCREASE)
                .onTrue(new InstantCommand(() -> wristSubsystem.setOffset(wristSubsystem.getOffset() + 1)));
        commandButtonPanel.button(ButtonPanelButtons.MOVE_WRIST_DECREASE)
                .onTrue(new InstantCommand(() -> wristSubsystem.setOffset(wristSubsystem.getOffset() - 1)));
        commandButtonPanel.button(ButtonPanelButtons.MOVE_ELEVATOR_INCREASE)
                .onTrue(new InstantCommand(() -> elevatorSubsystem.setOffset(elevatorSubsystem.getOffset() + 0.01)));
        commandButtonPanel.button(ButtonPanelButtons.MOVE_ELEVATOR_DECREASE)
                .onTrue(new InstantCommand(() -> elevatorSubsystem.setOffset(elevatorSubsystem.getOffset() - 0.01)));
        commandButtonPanel.button(ButtonPanelButtons.MOVE_ARM_INCREASE)
                .onTrue(new InstantCommand(() -> armSubsystem.setOffset(armSubsystem.getOffset() + 1)));
        commandButtonPanel.button(ButtonPanelButtons.MOVE_ARM_DECREASE)
                .onTrue(new InstantCommand(() -> armSubsystem.setOffset(armSubsystem.getOffset() - 1)));

        commandButtonPanel.button(ButtonPanelButtons.MOVE_CLIMB_INCREASE)
                .onTrue(new InstantCommand(() -> climberSubsystem.setPercent(1)))
                .onFalse(new InstantCommand(() -> climberSubsystem.setPercent(0)));
        commandButtonPanel.button(ButtonPanelButtons.MOVE_CLIMB_DECREASE)
                .onTrue(new InstantCommand(() -> climberSubsystem.setPercent(-1)))
                .onFalse(new InstantCommand(() -> climberSubsystem.setPercent(0)));

        commandButtonPanel.button(ButtonPanelButtons.AUX_LEFT)
                .onTrue(new InstantCommand(RobotContainer::toggleUseAutoAlign));
        commandButtonPanel.button(ButtonPanelButtons.AUX_RIGHT)
                .onTrue(new InstantCommand(RobotContainer::toggleShouldFixTip).alongWith(
                                new PositionCommand(elevatorSubsystem, .1))
                        .andThen(new PositionCommand(armSubsystem, 0))
                        .andThen(new PositionCommand(wristSubsystem, 60)));

//        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_A)
//                .onTrue(Autos.driveToPose(ScoringPosition.REEF_SIDE_A, 5, 5));
//        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_B)
//                .onTrue(Autos.driveToPose(ScoringPosition.REEF_SIDE_B, 5, 5));
//        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_C)
//                .onTrue(Autos.driveToPose(ScoringPosition.REEF_SIDE_C, 5, 5));
//        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_D)
//                .onTrue(Autos.driveToPose(ScoringPosition.REEF_SIDE_D, 5, 5));
//        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_E)
//                .onTrue(Autos.driveToPose(ScoringPosition.REEF_SIDE_E, 5, 5));
//        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_F)
//                .onTrue(Autos.driveToPose(ScoringPosition.REEF_SIDE_F, 5, 5));
//        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_G)
//                .onTrue(Autos.driveToPose(ScoringPosition.REEF_SIDE_G, 5, 5));
//        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_H)
//                .onTrue(Autos.driveToPose(ScoringPosition.REEF_SIDE_H, 5, 5));
//        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_I)
//                .onTrue(Autos.driveToPose(ScoringPosition.REEF_SIDE_I, 5, 5));
//        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_J)
//                .onTrue(Autos.driveToPose(ScoringPosition.REEF_SIDE_J, 5, 5));
//        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_K)
//                .onTrue(Autos.driveToPose(ScoringPosition.REEF_SIDE_K, 5, 5));
//        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_L)
//                .onTrue(Autos.driveToPose(ScoringPosition.REEF_SIDE_L, 5, 5));

        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_A)
                .onTrue(new InstantCommand(() -> currentScoringPosition = ScoringPosition.REEF_SIDE_A));
        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_B)
                .onTrue(new InstantCommand(() -> currentScoringPosition = ScoringPosition.REEF_SIDE_B));
        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_C)
                .onTrue(new InstantCommand(() -> currentScoringPosition = ScoringPosition.REEF_SIDE_C));
        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_D)
                .onTrue(new InstantCommand(() -> currentScoringPosition = ScoringPosition.REEF_SIDE_D));
        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_E)
                .onTrue(new InstantCommand(() -> currentScoringPosition = ScoringPosition.REEF_SIDE_E));
        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_F)
                .onTrue(new InstantCommand(() -> currentScoringPosition = ScoringPosition.REEF_SIDE_F));
        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_G)
                .onTrue(new InstantCommand(() -> currentScoringPosition = ScoringPosition.REEF_SIDE_G));
        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_H)
                .onTrue(new InstantCommand(() -> currentScoringPosition = ScoringPosition.REEF_SIDE_H));
        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_I)
                .onTrue(new InstantCommand(() -> currentScoringPosition = ScoringPosition.REEF_SIDE_I));
        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_J)
                .onTrue(new InstantCommand(() -> currentScoringPosition = ScoringPosition.REEF_SIDE_J));
        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_K)
                .onTrue(new InstantCommand(() -> currentScoringPosition = ScoringPosition.REEF_SIDE_K));
        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_L)
                .onTrue(new InstantCommand(() -> currentScoringPosition = ScoringPosition.REEF_SIDE_L));


        commandSwerveDrivetrain.registerTelemetry(logger::telemeterize);

        if (Utils.isSimulation()) {
            commandSwerveDrivetrain.resetPose(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }


        // Old Reef Controls


//        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_H).onTrue(ScoreCommands.Arm.armBarge());
//        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_G).onTrue(ScoreCommands.Arm.armProcessor());
//
//        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_A)
//                .onTrue(new InstantCommand(RobotContainer::setAutoAlignOffsetLeft));
//        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_B)
//                .onTrue(new InstantCommand(RobotContainer::setAutoAlignOffsetRight));
//
//        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_K)
//                .onTrue(new VelocityCommand(intakeSubsystem, 60. 60))
//                .onFalse(new VelocityCommand(intakeSubsystem, 0, 0));
//
//        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_D)
//                .onTrue(ScoreCommands.Intake.reIntakeSequence());
//
//        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_E)
//                .onTrue(ScoreCommands.Zeroing.zeroSubsystems());
//        commandButtonPanel.button(ButtonPanelButtons.REEF_SIDE_F)
//                .onTrue(new InstantCommand(() -> elevatorSubsystem.setPercent(0))
//                        .andThen(new InstantCommand(() -> armSubsystem.setPercent(0)))
//                        .andThen(new InstantCommand(() -> wristSubsystem.setPercent(0)))
//                        .andThen(new InstantCommand(() -> elevatorSubsystem.setFollowLastMechProfile(false)))
//                        .andThen(new InstantCommand(() -> armSubsystem.setFollowLastMechProfile(false)))
//                        .andThen(new InstantCommand(() -> wristSubsystem.setFollowLastMechProfile(false))));


        //Old operator controls


//        operatorXboxController.y().onTrue(new InstantCommand(()
//                -> armSubsystem.setOffset(armSubsystem.getOffset() + 1)));
//        operatorXboxController.x().onTrue(new InstantCommand(()
//                -> armSubsystem.setOffset(armSubsystem.getOffset() - 1)));

//        operatorXboxController.rightBumper().onTrue(new InstantCommand(()
//                -> elevatorSubsystem.setOffset(elevatorSubsystem.getOffset() + .01)));
//        operatorXboxController.leftBumper().onTrue(new InstantCommand(()
//                -> elevatorSubsystem.setOffset(elevatorSubsystem.getOffset() - .01)));

//        operatorXboxController.rightBumper().onTrue(new InstantCommand(this::setAutoAlignOffsetRight));
//        operatorXboxController.leftBumper().onTrue(new InstantCommand(this::setAutoAlignOffsetLeft));
//
//        operatorXboxController.b().onTrue(new InstantCommand(()
//                -> wristSubsystem.setOffset(wristSubsystem.getOffset() + 1)));
//        operatorXboxController.a().onTrue(new InstantCommand(()
//                -> wristSubsystem.setOffset(wristSubsystem.getOffset() - 1)));
//
//        operatorXboxController.povUp().onTrue(new InstantCommand(() -> climberSubsystem.setPercent(0.6)))
//                .onFalse(new InstantCommand(() -> climberSubsystem.setPercent(0)));
//        operatorXboxController.povDown().onTrue(new InstantCommand(() -> climberSubsystem.setPercent(-0.6)))
//                .onFalse(new InstantCommand(() -> climberSubsystem.setPercent(0)));
//        operatorXboxController.povRight().onTrue(new PositionCommand(wristSubsystem, 30));
//        operatorXboxController.povLeft().onTrue(new PositionCommand(wristSubsystem, 0));
//
//        operatorXboxController.rightTrigger().onTrue(new PositionCommand(elevatorSubsystem, .15)
//                .andThen(new PositionCommand(armSubsystem, 0).andThen(new PositionCommand(wristSubsystem, 0))));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    public Command getAutonomousCommand() {
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

        // System.out.println("X: " + aprilTagSubsystem.getClosestTagXYYaw()[0] + "Y: " + aprilTagSubsystem.getClosestTagXYYaw()[1]);

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

        double pitch = commandSwerveDrivetrain.getPigeon2().getPitch().getValueAsDouble();
        double roll = commandSwerveDrivetrain.getPigeon2().getRoll().getValueAsDouble();

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

        // // Code for if the bot starts tipping
        // if (((Math.abs(pitch) > 2 && Math.abs(pitch) < 90)
        //         || (Math.abs(roll) > 2 && Math.abs(roll) < 90)) && shouldFixTip) {
        //     commandSwerveDrivetrain.setControl(
        //             robotCentricDrive.withVelocityX(roll)
        //                     .withVelocityY(pitch));
        // }

        if (LimelightHelpers.getTV(Constants.Vision.LIMELIGHT_NAME)) {
            if (driveToPieceCurrentState.position < 5) {
                driveToPieceRotationVelocity = turnToPiecePIdControllerVeryClose
                        .calculate(LimelightHelpers.getTX(Constants.Vision.LIMELIGHT_NAME), -15);
            } else if (driveToPieceCurrentState.position < 20) {
                driveToPieceRotationVelocity = turnToPiecePIdControllerClose
                        .calculate(LimelightHelpers.getTX(Constants.Vision.LIMELIGHT_NAME), -15);
            } else {
                driveToPieceRotationVelocity = turnToPiecePIdController
                        .calculate(LimelightHelpers.getTX(Constants.Vision.LIMELIGHT_NAME), -15);
            }
            driveToPieceRotationVelocity = turnToPiecePIdController
                    .calculate(LimelightHelpers.getTX(Constants.Vision.LIMELIGHT_NAME), -10);
        }

//        driveToPieceCurrentState.velocity = commandSwerveDrivetrain.getState().Speeds.omegaRadiansPerSecond;
//        if (LimelightHelpers.getTV(Constants.Vision.LIMELIGHT_NAME)) {
//            driveToPieceCurrentState.position = LimelightHelpers.getTX(Constants.Vision.LIMELIGHT_NAME);
//        }
//
//        driveToPieceGoalState.position = 0;
//        driveToPieceGoalState.velocity = 0;
//
//        if (driveToPieceCurrentState.position < 5) {
//            driveToPieceRotationVelocity = turnToPiecePIdControllerVeryClose
//                    .calculate(driveToPieceCurrentState.position,
//                            driveToPieceRotation.calculate(.02, driveToPieceCurrentState, driveToPieceGoalState).position);
//        } else if (driveToPieceCurrentState.position < 20) {
//            driveToPieceRotationVelocity = turnToPiecePIdControllerClose
//                    .calculate(driveToPieceCurrentState.position,
//                            driveToPieceRotation.calculate(.02, driveToPieceCurrentState, driveToPieceGoalState).position);
//        } else {
//            driveToPieceRotationVelocity = turnToPiecePIdController
//                    .calculate(driveToPieceCurrentState.position,
//                            driveToPieceRotation.calculate(.02, driveToPieceCurrentState, driveToPieceGoalState).position);
//        }

//        System.out.println("Rotation velocity: " + driveToPieceRotationVelocity);


        // if (UserInterface.getControlComponent("Reset All").getBoolean(false)) {
        //     CommandScheduler.getInstance().schedule(ScoreCommands.Zeroing.zeroSubsystems());
        //     UserInterface.setControlComponent("Reset All", false);
        // } else if (UserInterface.getControlComponent("Reset Elevator").getBoolean(false)) {
        //     CommandScheduler.getInstance().schedule(ScoreCommands.Zeroing.zeroElevator());
        //     UserInterface.setControlComponent("Reset Elevator", false);
        // } else if (UserInterface.getControlComponent("Reset Arm").getBoolean(false)) {
        //     CommandScheduler.getInstance().schedule(ScoreCommands.Zeroing.zeroArm());
        //     UserInterface.setControlComponent("Reset Arm", false);
        // } else if (UserInterface.getControlComponent("Reset Wrist").getBoolean(false)) {
        //     CommandScheduler.getInstance().schedule(ScoreCommands.Zeroing.zeroWrist());
        //     UserInterface.setControlComponent("Reset Wrist", false);
        // }

        // UserInterface.setAutonComponent("Event", DriverStation.getEventName());
        // UserInterface.setAutonComponent("Game Message", DriverStation.getGameSpecificMessage());
        // UserInterface.setAutonComponent("Location", DriverStation.getLocation().getAsInt());
        // UserInterface.setAutonComponent("Alliance", DriverStation.getAlliance().get() == Alliance.Blue);
        // UserInterface.setAutonComponent("Enabled", DriverStation.isEnabled());
        // UserInterface.setAutonComponent("EStop", DriverStation.isEStopped());
        // UserInterface.setAutonComponent("Match Type", DriverStation.getMatchType().toString());
        // UserInterface.setAutonComponent("Match Number", DriverStation.getMatchNumber());
        // UserInterface.setAutonComponent("Replay Match Number", DriverStation.getReplayNumber());
        // UserInterface.setAutonComponent("Match Time", DriverStation.getMatchTime());

        // UserInterface.setTeleopComponent("Event", DriverStation.getEventName());
        // UserInterface.setTeleopComponent("Game Message", DriverStation.getGameSpecificMessage());
        // UserInterface.setTeleopComponent("Location", DriverStation.getLocation().getAsInt());
        // UserInterface.setTeleopComponent("Enabled", DriverStation.isEnabled());
        // UserInterface.setTeleopComponent("EStop", DriverStation.isEStopped());
        // UserInterface.setTeleopComponent("Alliance", DriverStation.getAlliance().get() == Alliance.Blue);
        // UserInterface.setTeleopComponent("Match Type", DriverStation.getMatchType().toString());
        // UserInterface.setTeleopComponent("Match Number", DriverStation.getMatchNumber());
        // UserInterface.setTeleopComponent("Replay Match Number", DriverStation.getReplayNumber());
        // UserInterface.setTeleopComponent("Match Time", DriverStation.getMatchTime());

//         UserInterface.setControlComponent("Reset All", ScoreCommands.Zeroing.zeroSubsystems());
//         UserInterface.setControlComponent("Reset Elevator", ScoreCommands.Zeroing.zeroElevator());
//         UserInterface.setControlComponent("Reset Arm", ScoreCommands.Zeroing.zeroArm());
//         UserInterface.setControlComponent("Reset Wrist", ScoreCommands.Zeroing.zeroWrist());
//         UserInterface.setControlComponent("Setpoint L1", ScoreCommands.Score.scoreL1());
//         UserInterface.setControlComponent("Setpoint L2", ScoreCommands.Score.scoreL2());
//         UserInterface.setControlComponent("Setpoint L3", ScoreCommands.Score.scoreL3());
//         UserInterface.setControlComponent("Setpoint L4", ScoreCommands.Score.scoreL4());
//         UserInterface.setControlComponent("Setpoint Ground Intake", ScoreCommands.Intake.intakeGround());
//         UserInterface.setControlComponent("Setpoint Human Player", ScoreCommands.Intake.intakeHP());
//         UserInterface.setControlComponent("Setpoint Algae High", ScoreCommands.Score.removeAlgaeHigh());
//         UserInterface.setControlComponent("Setpoint Algae Low", ScoreCommands.Score.removeAlgaeLow());

//        System.out.println("aligned: " + aligned());
//        System.out.println("X speed: " + commandSwerveDrivetrain.getState().Speeds.vxMetersPerSecond
//                + " Y: " + commandSwerveDrivetrain.getState().Speeds.vyMetersPerSecond);

//        System.out.println("Pose: " + commandSwerveDrivetrain.getPose());
//        System.out.println(selectedReefTag);
//        System.out.println(lockOnMode);
//
//        System.out.println("Drive: " + commandSwerveDrivetrain.getPose().getRotation().getDegrees());
//        System.out.println("Pigeon: " + commandSwerveDrivetrain.getPigeon2().getRotation2d().getDegrees());

//        System.out.println("Elevator: " + elevatorSubsystem.getMechM());
//        System.out.println("Arm: " + armSubsystem.getDegrees());
//        System.out.println("Wrist: " + wristSubsystem.getDegrees());

//        System.out.println("Elevator goal: " + elevatorSubsystem.getGoal());
//        System.out.println("Wrist goal: " + wristSubsystem.getGoal());
//        System.out.println("Arm goal: " + armSubsystem.getGoal());

//        System.out.println("Wrist at goal: " + wristSubsystem.isMechAtGoal(false));

//        System.out.println("Intake Velocity: " + intakeSubsystem.getMechVelocity());

//        System.out.println("Has Coral: " + intakeSubsystem.hasCoral());

//        System.out.println("Robot Pitch: " + commandSwerveDrivetrain.getPigeon2().getPitch().getValueAsDouble());
//        System.out.println("Robot Roll: " + commandSwerveDrivetrain.getPigeon2().getRoll().getValueAsDouble());
    }

    public static void setAutoAlignOffsetLeft() {
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

    public static void setAutoAlignOffsetRight() {
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
        return Math.abs(aprilTagSubsystem.getClosestTagXYYaw()[0] - autoAlignXOffset) <= .025
                && Math.abs(aprilTagSubsystem.getClosestTagXYYaw()[1] - autoAlignYOffset) <= .02;
    }

    public static void setState(State state) {
        RobotContainer.state = state;
    }

    public static State getState() {
        return state;
    }

    public static void toggleUseAutoAlign() {
        useAutoAlign = !useAutoAlign;
    }

    public static boolean isUseAutoAlign() {
        return useAutoAlign;
    }

    public static void toggleShouldFixTip() {
        shouldFixTip = !shouldFixTip;
    }

    public static void toggleCoralBlockingHP() {
        isCoralBlockingHP = !isCoralBlockingHP;
    }
}
