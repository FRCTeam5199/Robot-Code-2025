// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Autos;
import frc.robot.commands.ScoreCommands;
import frc.robot.commands.ScoreCommands.Score;
import frc.robot.constants.Constants;
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
    public static double MaxAngularRate = 2.5 * Math.PI; //Originally 2 * Math.PI

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
    private static final ProfiledPIDController drivePIDControllerXClose = new ProfiledPIDController(5, 0, .15, new TrapezoidProfile.Constraints(100, 200));

    private static final ProfiledPIDController drivePIDControllerY = new ProfiledPIDController(3, 0, .05, new TrapezoidProfile.Constraints(100, 200));
    private static final ProfiledPIDController drivePIDControllerYClose = new ProfiledPIDController(8, 0.0, .25, new TrapezoidProfile.Constraints(100, 200));
    private static final ProfiledPIDController drivePIDControllerYVeryClose = new ProfiledPIDController(9, 0.0, .25, new TrapezoidProfile.Constraints(100, 200));

    public static final ProfiledPIDController turnPIDController = new ProfiledPIDController(0.175, 0.0, 0.0, new TrapezoidProfile.Constraints(100, 200));

    public static double xVelocity = 0;
    public static double yVelocity = 0;
    public static double rotationVelocity = 0;

    public static double autoAlignXOffset = Constants.Vision.AUTO_ALIGN_X;
    public static double autoAlignYOffset = Constants.Vision.AUTO_ALIGN_Y;

    public static final CommandSwerveDrivetrain commandSwerveDrivetrain = TunerConstants.createDrivetrain();
    private static final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private static final ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();
    private static final WristSubsystem wristSubsystem = WristSubsystem.getInstance();
    private static final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    private static final ClimberSubsystem climberSubsystem = ClimberSubsystem.getInstance();

    public static final AprilTagSubsystem aprilTagSubsystem = AprilTagSubsystem.getInstance();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    public static State state = State.L4;
    private static Timer timer = new Timer();
    private static boolean useAutoAlign = true;
    private static boolean shouldFixTip = true;
    private static boolean isCoralBlockingHP = false;
    private static boolean backwardsAlgae = false;
    private static boolean isIntaking = false;
    private static boolean autoRetractIntake = true;

    public static ScoringPosition getCurrentScoringPosition() {
        return currentScoringPosition;
    }

    private static ScoringPosition currentScoringPosition = ScoringPosition.REEF_SIDE_A;

    private static int readyToAlignCheck = 0;

    private static boolean readyToAlign = false;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        NamedCommands.registerCommand("ARML4", ScoreCommands.Arm.armL4());
        NamedCommands.registerCommand("ARML2", ScoreCommands.Arm.armL2());
        NamedCommands.registerCommand("PREPL4", ScoreCommands.Score.prepL4());
        NamedCommands.registerCommand("UNWIND", ScoreCommands.Climber.slightUnwindAuton()
                .andThen(ScoreCommands.Arm.armL4()));
        NamedCommands.registerCommand("UNWINDL4", ScoreCommands.Climber.slightUnwindAuton()
                .andThen(ScoreCommands.Arm.armL4())
                .andThen(ScoreCommands.Score.scoreL4()));
        NamedCommands.registerCommand("L4", Score.scoreL4().withTimeout(3));
        NamedCommands.registerCommand("L2", Score.scoreL2().withTimeout(3));
        NamedCommands.registerCommand("HP", ScoreCommands.Intake.intakeHP());
        NamedCommands.registerCommand("INTAKEGROUNDPREP", ScoreCommands.Intake.intakeGroundPrep());


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
                .until(() -> !intakeSubsystem.hasCoral())
                .withTimeout(2)
                .andThen(new InstantCommand(() -> wristSubsystem.setPosition(Constants.WristConstants.HP))));

        NamedCommands.registerCommand("DROP", ScoreCommands.Climber.drop());

        NamedCommands.registerCommand("GROUNDINTAKESEQUENCE",
                ScoreCommands.Intake.intakeGround());
        NamedCommands.registerCommand("DRIVEINTAKE", ScoreCommands.Drive.driveForward()
                .alongWith(new VelocityCommand(intakeSubsystem, 120, 120))
                .until(intakeSubsystem::hasCoral)
                .andThen(ScoreCommands.Stabling.groundIntakeStable())
                .withTimeout(1.6));

        Autos.initializeAutos();

        configureBindings();
        SignalLogger.setPath("/media/BRAZIL/");
        SignalLogger.writeBoolean("Camera Present", aprilTagSubsystem.camera.isConnected());
        SignalLogger.writeDouble("X Value", aprilTagSubsystem.updateClosestTagXYYaw()[0]);
        SignalLogger.writeDouble("Y Value", aprilTagSubsystem.updateClosestTagXYYaw()[1]);

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

        commandXboxController.leftTrigger().onTrue(
                        new ConditionalCommand(
                                new ConditionalCommand(
                                        ScoreCommands.Score.score()
                                                .alongWith(new InstantCommand(() -> intakeSubsystem.setIntakeMotors(120, 120)))
                                                .alongWith(commandSwerveDrivetrain.applyRequest(() -> drive
                                                        .withVelocityX(-commandXboxController.getLeftY() * MaxSpeed)
                                                        .withVelocityY(-commandXboxController.getLeftX() * MaxSpeed)
                                                        .withRotationalRate(-commandXboxController.getRightX() * MaxAngularRate))),
                                        ScoreCommands.Score.score()
                                                .alongWith(new VelocityCommand(intakeSubsystem, 120, 120))
                                                .alongWith(commandSwerveDrivetrain.applyRequest(() -> drive
                                                        .withVelocityX(-commandXboxController.getLeftY() * MaxSpeed)
                                                        .withVelocityY(-commandXboxController.getLeftX() * MaxSpeed)
                                                        .withRotationalRate(-commandXboxController.getRightX() * MaxAngularRate))),
                                        () -> state == State.ALGAE_LOW || state == State.ALGAE_HIGH
                                ),
                                ScoreCommands.Score.score()
                                        .alongWith(ScoreCommands.Drive.autoAlignLAuton())
                                        .andThen(ScoreCommands.Score.place().unless(() -> state == State.L1 || state == State.L1_UP || !isUseAutoAlign())),
                                () -> (state == State.BARGE
                                        || state == State.PROCESSOR
                                        || state == State.ALGAE_LOW
                                        || state == State.ALGAE_HIGH)
                        ).alongWith(new InstantCommand(() -> aprilTagSubsystem.setDoneAutoDriving(true))))
                .onFalse(ScoreCommands.Stabling.stable()
                        .alongWith(new VelocityCommand(intakeSubsystem, 120, 120)
                                .onlyIf(() -> state == State.ALGAE_LOW || state == State.ALGAE_HIGH))
                        .alongWith(commandSwerveDrivetrain.applyRequest(() -> drive
                                .withVelocityX(-commandXboxController.getLeftY() * MaxSpeed)
                                .withVelocityY(-commandXboxController.getLeftX() * MaxSpeed)
                                .withRotationalRate(-commandXboxController.getRightX() * MaxAngularRate))));

        commandXboxController.rightTrigger().onTrue(
                        new ConditionalCommand(
                                new ConditionalCommand(
                                        ScoreCommands.Score.score()
                                                .alongWith(new InstantCommand(() -> intakeSubsystem.setIntakeMotors(120, 120)))
                                                .alongWith(commandSwerveDrivetrain.applyRequest(() -> drive
                                                        .withVelocityX(-commandXboxController.getLeftY() * MaxSpeed)
                                                        .withVelocityY(-commandXboxController.getLeftX() * MaxSpeed)
                                                        .withRotationalRate(-commandXboxController.getRightX() * MaxAngularRate))),
                                        ScoreCommands.Score.score()
                                                .alongWith(new VelocityCommand(intakeSubsystem, 120, 120))
                                                .alongWith(commandSwerveDrivetrain.applyRequest(() -> drive
                                                        .withVelocityX(-commandXboxController.getLeftY() * MaxSpeed)
                                                        .withVelocityY(-commandXboxController.getLeftX() * MaxSpeed)
                                                        .withRotationalRate(-commandXboxController.getRightX() * MaxAngularRate))),
                                        () -> state == State.ALGAE_LOW || state == State.ALGAE_HIGH
                                ),
                                ScoreCommands.Score.score()
                                        .alongWith(ScoreCommands.Drive.autoAlignRAuton())
                                        .andThen(ScoreCommands.Score.place().unless(() -> state == State.L1 || state == State.L1_UP || !isUseAutoAlign())),
                                () -> (state == State.BARGE
                                        || state == State.PROCESSOR
                                        || state == State.ALGAE_LOW
                                        || state == State.ALGAE_HIGH)
                        ).alongWith(new InstantCommand(() -> aprilTagSubsystem.setDoneAutoDriving(true))))
                .onFalse(ScoreCommands.Stabling.stable()
                        .alongWith(new VelocityCommand(intakeSubsystem, 120, 120)
                                .onlyIf(() -> state == State.ALGAE_LOW || state == State.ALGAE_HIGH))
                        .alongWith(commandSwerveDrivetrain.applyRequest(() -> drive
                                .withVelocityX(-commandXboxController.getLeftY() * MaxSpeed)
                                .withVelocityY(-commandXboxController.getLeftX() * MaxSpeed)
                                .withRotationalRate(-commandXboxController.getRightX() * MaxAngularRate))));

        commandXboxController.leftBumper().onTrue(
                        new ConditionalCommand(
                                ScoreCommands.Intake.intakeGroundAlgae()
                                        .alongWith(commandSwerveDrivetrain.applyRequest(() -> drive
                                                .withVelocityX(-commandXboxController.getLeftY() * MaxSpeed * .7)
                                                .withVelocityY(-commandXboxController.getLeftX() * MaxSpeed * .7)
                                                .withRotationalRate(-commandXboxController.getRightX() * MaxAngularRate * .7))),
                                ScoreCommands.Intake.intakeGround()
                                        .alongWith(new VelocityCommand(intakeSubsystem, 120, 90))
                                        .alongWith(commandSwerveDrivetrain.applyRequest(() -> drive
                                                .withVelocityX(-commandXboxController.getLeftY() * MaxSpeed * .7)
                                                .withVelocityY(-commandXboxController.getLeftX() * MaxSpeed * .7)
                                                .withRotationalRate(-commandXboxController.getRightX() * MaxAngularRate * .7)))
                                        .until(() -> intakeSubsystem.hasCoral() && autoRetractIntake)
                                        .andThen(ScoreCommands.Stabling.groundIntakeStable()),
                                () -> state == State.BARGE || state == State.PROCESSOR
                                        || state == State.ALGAE_LOW || state == State.ALGAE_HIGH
                        ).alongWith(new InstantCommand(() -> isIntaking = true)))
                .onFalse(new ConditionalCommand(
                        ScoreCommands.Arm.armStable()
                                .alongWith(new VelocityCommand(intakeSubsystem, 120, 120)),
                        ScoreCommands.Stabling.groundIntakeStable(),
                        () -> state == State.BARGE || state == State.PROCESSOR
                ).alongWith(new InstantCommand(() -> isIntaking = false)));

        commandXboxController.rightBumper().onTrue(ScoreCommands.Score.place())
                .onFalse(
                        new ConditionalCommand(
                                new VelocityCommand(intakeSubsystem, 30, 30),
                                new VelocityCommand(intakeSubsystem, 0, 0),
                                () -> RobotContainer.getState() == State.BARGE
                                        || RobotContainer.getState() == State.PROCESSOR
                        ));
        commandXboxController.button(7).onTrue(ScoreCommands.Zeroing.zeroSubsystems());

        commandXboxController.povRight().onTrue(new VelocityCommand(intakeSubsystem, -120, -120))
                .onFalse(new VelocityCommand(intakeSubsystem, 0, 0)); //top right button

        commandXboxController.povDown().onTrue(ScoreCommands.Arm.armAlgaeHigh()); //right top paddle
        commandXboxController.button(10).onTrue(ScoreCommands.Arm.armAlgaeLow()); //right bottom paddle

        commandXboxController.povUp().onTrue(ScoreCommands.Arm.armBarge()
                .alongWith(new VelocityCommand(intakeSubsystem, 120, 120))); //left top paddle
        commandXboxController.button(9).onTrue(ScoreCommands.Arm.armL1Up());

//        commandXboxController.povUp().onTrue(new SequentialCommandGroup(
//                        ScoreCommands.Drive.driveToPiece()
//                                .alongWith(ScoreCommands.Intake.intakeGroundPrep())
//                                .until(() -> LimelightHelpers.getTY(Constants.Vision.LIMELIGHT_NAME) < 13),
//                        ScoreCommands.Drive.driveForward()
//                                .alongWith(ScoreCommands.Intake.intakeGround())
//                                .until(intakeSubsystem::hasCoral),
//                        commandSwerveDrivetrain.applyRequest(() -> drive
//                                        .withVelocityX(-commandXboxController.getLeftY() * MaxSpeed)
//                                        .withVelocityY(-commandXboxController.getLeftX() * MaxSpeed)
//                                        .withRotationalRate(-commandXboxController.getRightX() * MaxAngularRate))
//                                .alongWith(ScoreCommands.Stabling.groundIntakeStable())
//                ))
//                .onFalse(commandSwerveDrivetrain.applyRequest(() -> drive
//                                .withVelocityX(-commandXboxController.getLeftY() * MaxSpeed)
//                                .withVelocityY(-commandXboxController.getLeftX() * MaxSpeed)
//                                .withRotationalRate(-commandXboxController.getRightX() * MaxAngularRate))
//                        .alongWith(ScoreCommands.Stabling.groundIntakeStable()));

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
                .onTrue(new VelocityCommand(intakeSubsystem, -120, -120))
                .onFalse(new VelocityCommand(intakeSubsystem, 0, 0));

        commandButtonPanel.button(ButtonPanelButtons.REEF_SCORE_L2)
                .onTrue(new VelocityCommand(intakeSubsystem, 120, 120));
        commandButtonPanel.button(ButtonPanelButtons.REEF_SCORE_L4)
                .onTrue(new InstantCommand(() -> autoRetractIntake = !autoRetractIntake));

        commandButtonPanel.button(ButtonPanelButtons.SETMODE_ALGAE)
                .onTrue(ScoreCommands.Arm.armAlgaeHigh().beforeStarting(new InstantCommand(() -> backwardsAlgae = true)));
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
//                .onTrue(new VelocityCommand(intakeSubsystem, 60. 60)
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
        if (Robot.getAlliance().equals(DriverStation.Alliance.Red)) {
            return Autos.autonChooserRed.getSelected();
        } else {
            return Autos.autonChooserBlue.getSelected();
        }
    }

    public static void periodic() {
        if (readyToAlign()) {
            readyToAlignCheck++;
            readyToAlign = readyToAlignCheck >= 1;
        } else {
            readyToAlignCheck = 0;
            readyToAlign = false;
        }

//        System.out.println("X: " + aprilTagSubsystem.updateClosestTagXYYaw()[0]);
//        System.out.println("Id: " + aprilTagSubsystem.getClosestTagID());
//        System.out.println("Ready to Align: " + isReadyToAlign());

        double currentX = aprilTagSubsystem.getClosestTagX();
        double currentY = aprilTagSubsystem.getClosestTagY();
        double currentRotation = commandSwerveDrivetrain.getPose().getRotation().getDegrees();

        if ((Math.abs(currentX - autoAlignXOffset) > .15
                || Math.abs(currentY - autoAlignYOffset) > .1)) {
            xVelocity = drivePIDControllerX.calculate(currentX, autoAlignXOffset);
            yVelocity = drivePIDControllerY.calculate(currentY, autoAlignYOffset);
        } else if (Math.abs(currentY - autoAlignYOffset) > .04) {
            xVelocity = drivePIDControllerXClose.calculate(currentX, autoAlignXOffset);
            yVelocity = drivePIDControllerYClose.calculate(currentY, autoAlignYOffset);
        } else {
            xVelocity = drivePIDControllerXClose.calculate(currentX, autoAlignXOffset);
            yVelocity = drivePIDControllerYVeryClose.calculate(currentY, autoAlignYOffset);
        }
        rotationVelocity = turnPIDController.calculate(currentRotation, 0);

//        System.out.println("Front X: " + aprilTagSubsystem.updateClosestTagXYYaw()[0]
//                + " Front Y: " + aprilTagSubsystem.updateClosestTagXYYaw()[1]);
        //0, .17; 0, .175; 0, .165, 0, .17; 0, .16;, 0, .17; 0, .17; 0, .175; 0, .165;
        // 0, .175
//        System.out.println("X Offset: " + autoAlignXOffset + " Y Offset: " + autoAlignYOffset);

//        System.out.println("Back X: " + aprilTagSubsystem.getBackClosestTagXYYaw()[0]
//                + " Back Y: " + aprilTagSubsystem.getBackClosestTagXYYaw()[1]);
//
//        System.out.println("X velocity: " + xVelocity);
//        System.out.println("Y velocity: " + yVelocity);
//        System.out.println("X Offset: " + autoAlignXOffset);
//        System.out.println("X Offset Manual: " + autoAlignXOffsetManual);
//        System.out.println("should align backwards manual: " + shouldAlignBackwardsManual);
//        System.out.println("Closest Tag ID: " + aprilTagSubsystem.getClosestTagID());
//        System.out.println("Closest Tag ID Manual: " + aprilTagSubsystem.getClosestTagIDManual());

//        if (LimelightHelpers.getTV(Constants.Vision.LIMELIGHT_NAME)) {
//            double currentX = LimelightHelpers.getTX(Constants.Vision.LIMELIGHT_NAME);
//            driveToPieceRotationVelocity = turnToPiecePIdController
//                    .calculate(currentX, Constants.Vision.LIMELIGHT_X_AIM);
//        }

//        driveToPieceCurrentState.velocity = commandSwerveDrivetrain.getState().Speeds.omegaRadiansPerSecond;
//        if (LimelightHelpers.getTV(Constants.Vision.LIMELIGHT_NAME)) {
//            driveToPieceCurrentState.position = LimelightHelpers.getTX(Constants.Vision.LIMELIGHT_NAME);
//        }


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
        if (Robot.getAlliance().equals(DriverStation.Alliance.Blue)) {
            autoAlignYOffset = Constants.Vision.AUTO_ALIGN_Y;
        } else {
            autoAlignYOffset = -Constants.Vision.AUTO_ALIGN_Y;
        }
        autoAlignXOffset = Constants.Vision.AUTO_ALIGN_X;
        if (Robot.getAlliance().equals(DriverStation.Alliance.Red)) {
            autoAlignXOffset = -autoAlignXOffset;
        }

        if (Robot.getAlliance().equals(DriverStation.Alliance.Blue)
                && aprilTagSubsystem.getClosestTagID() == 19) {
            autoAlignYOffset -= .04;
        }
    }

    public static void setAutoAlignOffsetRight() {
        if (Robot.getAlliance().equals(DriverStation.Alliance.Blue)) {
            autoAlignYOffset = -Constants.Vision.AUTO_ALIGN_Y;
        } else {
            autoAlignYOffset = Constants.Vision.AUTO_ALIGN_Y;
        }
        autoAlignXOffset = Constants.Vision.AUTO_ALIGN_X;
        if (Robot.getAlliance().equals(DriverStation.Alliance.Red)) {
            autoAlignXOffset = -autoAlignXOffset;
        }
    }

    public static void setAutoAlignOffsetCenter() {
        autoAlignYOffset = 0;
        autoAlignXOffset = Constants.Vision.AUTO_ALIGN_X;
        if (Robot.getAlliance().equals(DriverStation.Alliance.Red)) {
            autoAlignXOffset = -autoAlignXOffset;
        }
    }

    public static void setAutoAlignOffsetCenterBack() {
        autoAlignYOffset = 0;
        autoAlignXOffset = (Robot.getAlliance().equals(DriverStation.Alliance.Blue)) ?
                Constants.Vision.AUTO_ALIGN_X_ALGAE_PREP : -Constants.Vision.AUTO_ALIGN_X_ALGAE_PREP;
    }

    public void toggleAutoAlignOffset() {
        autoAlignYOffset = -autoAlignYOffset;
    }

    public static boolean readyToAlign() {
        return aprilTagSubsystem.getClosestTagID() ==
                (Robot.getAlliance().equals(DriverStation.Alliance.Blue)
                        ? currentScoringPosition.getBlueAprilTagID()
                        : currentScoringPosition.getRedAprilTagID())
                && aprilTagSubsystem.updateClosestTagXYYaw()[0] != 0;
    }

    public static boolean isReadyToAlign() {
        return readyToAlign;
    }

    public static boolean isReadyToScore() {
        return Math.sqrt(Math.pow(aprilTagSubsystem.getClosestTagX(), 2)
                + Math.pow(aprilTagSubsystem.getClosestTagY(), 2)) < 1d;
    }

    public static boolean aligned() {
        return Math.abs(aprilTagSubsystem.getClosestTagX() - autoAlignXOffset) <= .025
                && Math.abs(aprilTagSubsystem.getClosestTagY() - autoAlignYOffset) <= .02;
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

    public static boolean isIntaking() {
        return isIntaking;
    }
}
