/// / Copyright (c) FIRST and other WPILib contributors.
/// / Open Source Software; you can modify and/or share it under the terms of
/// / the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.UserInterface;
import frc.robot.constants.Constants;
import frc.robot.subsystems.*;
import frc.robot.utility.ScoringPosition;
import frc.robot.utility.State;

import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;

public final class Autos {
    private SendableChooser<Command> autoChooser;
    private static Autos autos;

    public static SendableChooser<Command> autonChooserRed = new SendableChooser<>();
    public static SendableChooser<Command> autonChooserBlue = new SendableChooser<>();

    private static PathPlannerAuto onePieceBlueDropL1;
    private static PathPlannerAuto onePieceRedDropL1;

    private static PathPlannerAuto onePieceBlueRightL4;
    private static PathPlannerAuto onePieceBlueLeftL4;

    private static PathPlannerAuto onePieceRedRightL4;
    private static PathPlannerAuto onePieceRedLeftL4;

    private static PathPlannerAuto twoPieceBlueBottomL4;
    private static PathPlannerAuto twoPieceBlueTopL4;

    private static PathPlannerAuto twoPieceRedBottomL4;
    private static PathPlannerAuto twoPieceRedTopL4;

    private static PathPlannerAuto testBlue;

    private static PathPlannerAuto threePieceBlueBottomL4;
    private static PathPlannerAuto threePieceBlueTopL4;

    private static PathPlannerAuto threePieceRedBottomL4;
    private static PathPlannerAuto threePieceRedTopL4;

    private static Command threePieceFrontRightBlue;
    private static Command threePieceFrontLeftBlue;

    private static Command threePieceFrontRightRed;
    private static Command threePieceFrontLeftRed;

    private static PathPlannerAuto fourPieceBlueBottomL4;

    private static PathPlannerAuto helper2PieceTopBlue;
    private static PathPlannerAuto helper2PieceBottomRed;

    private static Command algaeBlue;
    private static Command algaeRed;

    private static final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    private static final WristSubsystem wristSubsystem = WristSubsystem.getInstance();
    private static final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private static final CommandSwerveDrivetrain commandSwerveDrivetrain = RobotContainer.commandSwerveDrivetrain;
    private static final AprilTagSubsystem aprilTagSubsystem = AprilTagSubsystem.getInstance();
    private static final ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();

    public final static SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDesaturateWheelSpeeds(true)
            .withDeadband(RobotContainer.MaxSpeed * .05).withRotationalDeadband(RobotContainer.MaxAngularRate * .05) // Add a 10% deadband
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);


    /**
     * Gets or creates the AutoChooser (Singleton Method)
     */
    public SendableChooser<Command> getAutoChooser() {
        if (autoChooser == null) {
            autoChooser = AutoBuilder.buildAutoChooser();
            UserInterface.getTab("Auton").add("AutoChooser", autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withSize(1, 1).withPosition(0, 0);
        }

        return autoChooser;
    }

    //Blue - Top is Left, Bottom is Right
    //Red - Top is Right, Bottom is Left
    public static void initializeAutos() {
//        onePieceBlueDropL1 = new PathPlannerAuto("Blue Climber Drop");
//        onePieceRedDropL1 = new PathPlannerAuto("Red Climber Drop");
//
//        onePieceBlueRightL4 = new PathPlannerAuto("1 Piece Blue Right L4");
//        onePieceBlueLeftL4 = new PathPlannerAuto("1 Piece Blue Left L4");
//        onePieceRedRightL4 = new PathPlannerAuto("1 Piece Red Right L4");
//        onePieceRedLeftL4 = new PathPlannerAuto("1 Piece Red Left L4");
//
//        helper2PieceTopBlue = new PathPlannerAuto("2 Piece Blue Top L4 Helper");
//        helper2PieceBottomRed = new PathPlannerAuto("2 Piece Red Bottom L4 Helper");
//
//        twoPieceBlueBottomL4 = new PathPlannerAuto("2 Piece Blue Bottom L4");
//        twoPieceBlueTopL4 = new PathPlannerAuto("2 Piece Blue Top L4");
//        twoPieceRedBottomL4 = new PathPlannerAuto("2 Piece Red Bottom L4");
//        twoPieceRedTopL4 = new PathPlannerAuto("2 Piece Red Top L4");
//
//        threePieceBlueBottomL4 = new PathPlannerAuto("3 Piece Blue Bottom L4");
//        threePieceBlueTopL4 = new PathPlannerAuto("3 Piece Blue Top L4");
//        threePieceRedBottomL4 = new PathPlannerAuto("3 Piece Red Bottom L4");
//        threePieceRedTopL4 = new PathPlannerAuto("3 Piece Red Top L4");
//
        threePieceFrontRightBlue = threePieceFrontRightBlue();
        threePieceFrontLeftBlue = threePieceFrontLeftBlue();

        threePieceFrontRightRed = threePieceFrontRightRed();
        threePieceFrontLeftRed = threePieceFrontLeftRed();

//        fourPieceBlueBottomL4 = new PathPlannerAuto("4 Piece Blue Bottom L4");

        testBlue = new PathPlannerAuto("Test Blue");

        algaeBlue = algaeBlue();
        algaeRed = algaeRed();

        Shuffleboard.getTab("Autons").add("Red Autons", autonChooserRed)
                .withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0)
                .withSize(2, 1);
        Shuffleboard.getTab("Autons").add("Blue Autons", autonChooserBlue)
                .withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0)
                .withSize(2, 1);

//        autonChooserRed.addOption("1 Piece Red Climber Drop", onePieceRedDropL1);
//        autonChooserRed.addOption("1 Piece Red Right", onePieceRedRightL4);
//        autonChooserRed.addOption("1 Piece Red Left", onePieceRedLeftL4);
//        autonChooserRed.addOption("2 Piece Red Left", twoPieceRedBottomL4);
//        autonChooserRed.addOption("2 Piece Red Right", twoPieceRedTopL4);
//        autonChooserRed.addOption("3 Piece Red Left", threePieceRedBottomL4);
//        autonChooserRed.addOption("3 Piece Red Right", threePieceRedTopL4);
//        autonChooserRed.addOption("2 Piece Red Left Helper", helper2PieceBottomRed);
        autonChooserRed.addOption("Algae Red", algaeRed);
        autonChooserRed.addOption("3 Piece Red Floor Right", threePieceFrontRightRed);
        autonChooserRed.addOption("3 Piece Red Floor Left", threePieceFrontLeftRed);

//        autonChooserBlue.addOption("1 Piece Blue Climber Drop", onePieceBlueDropL1);
//        autonChooserBlue.addOption("1 Piece Blue Right", onePieceBlueRightL4);
//        autonChooserBlue.addOption("1 Piece Blue Left", onePieceBlueLeftL4);
//        autonChooserBlue.addOption("2 Piece Blue Left", twoPieceBlueTopL4);
//        autonChooserBlue.addOption("2 Piece Blue Right", twoPieceBlueBottomL4);
//        autonChooserBlue.addOption("3 Piece Blue Left", threePieceBlueTopL4);
//        autonChooserBlue.addOption("3 Piece Blue Right", threePieceBlueBottomL4);
//        autonChooserBlue.addOption("2 Piece Blue Left Helper", helper2PieceTopBlue);
        autonChooserBlue.addOption("Algae Blue", algaeBlue);
        autonChooserBlue.addOption("3 Piece Blue Floor Right", threePieceFrontRightBlue);
        autonChooserBlue.addOption("3 Piece Blue Floor Left", threePieceFrontLeftBlue);
//        autonChooserBlue.addOption("Test", testBlue);
    }

    public static Command driveToPose(double goalX, double goalY, double goalDegrees) {
        return new SequentialCommandGroup(
                new InstantCommand(() ->
                        commandSwerveDrivetrain.applyRequest(() ->
                                drive.withVelocityX(0).
                                        withVelocityX(0).
                                        withRotationalRate(0)
                        )
                ),
                AutoBuilder.pathfindToPose(
                        new Pose2d(goalX, goalY, Rotation2d.fromDegrees(goalDegrees)),
                        new PathConstraints(2d, 2d, Units.degreesToRadians(540d), Units.degreesToRadians(720d)),
                        0d
                )
        );
    }

    public static Command driveToPose(ScoringPosition scoringPosition) {
        return new SequentialCommandGroup(
                new InstantCommand(() ->
                        commandSwerveDrivetrain.applyRequest(() ->
                                drive.withVelocityX(0).
                                        withVelocityX(0).
                                        withRotationalRate(0)
                        )
                ),
                new ConditionalCommand(
                        AutoBuilder.pathfindToPose(
                                scoringPosition.getBluePose(),
                                new PathConstraints(5.5, 5.5,
                                        Units.degreesToRadians(540d), Units.degreesToRadians(720d)), 0d),
                        AutoBuilder.pathfindToPose(
                                scoringPosition.getRedPose(),
                                new PathConstraints(5.5, 5.5,
                                        Units.degreesToRadians(540d), Units.degreesToRadians(720d)), 0d),
                        () -> DriverStation.getAlliance().isPresent()
                                && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)
                )
        );
    }

    public static Command driveToPoseChoice(ScoringPosition scoringPosition) {
        return new SequentialCommandGroup(
                new InstantCommand(() ->
                        commandSwerveDrivetrain.applyRequest(() ->
                                drive.withVelocityX(0).
                                        withVelocityX(0).
                                        withRotationalRate(0)
                        )
                ),
                new ConditionalCommand(
                        new ConditionalCommand(
                                AutoBuilder.pathfindToPose(
                                        scoringPosition.getBluePose().plus(new Transform2d(0, 0, new Rotation2d(Math.toRadians(180)))),
                                        new PathConstraints(5.5, 5.5,
                                                Units.degreesToRadians(540d), Units.degreesToRadians(720d)), 0d),
                                AutoBuilder.pathfindToPose(
                                        scoringPosition.getRedPose().plus(new Transform2d(0, 0, new Rotation2d(Math.toRadians(180)))),
                                        new PathConstraints(5.5, 5.5,
                                                Units.degreesToRadians(540d), Units.degreesToRadians(720d)), 0d),
                                () -> DriverStation.getAlliance().isPresent()
                                        && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)
                        ),
                        new ConditionalCommand(
                                AutoBuilder.pathfindToPose(
                                        scoringPosition.getBluePose(),
                                        new PathConstraints(5.5, 5.5,
                                                Units.degreesToRadians(540d), Units.degreesToRadians(720d)), 0d),
                                AutoBuilder.pathfindToPose(
                                        scoringPosition.getRedPose(),
                                        new PathConstraints(5.5, 5.5,
                                                Units.degreesToRadians(540d), Units.degreesToRadians(720d)), 0d),
                                () -> DriverStation.getAlliance().isPresent()
                                        && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)
                        ),
                        RobotContainer::getShouldAlignBackwards
                )
        );
    }

    public static Command driveToPose(Supplier<ScoringPosition> scoringPositionSupplier) {
        return new SequentialCommandGroup(
                new InstantCommand(() ->
                        commandSwerveDrivetrain.applyRequest(() ->
                                drive.withVelocityX(0).
                                        withVelocityX(0).
                                        withRotationalRate(0)
                        )
                ),
                new ConditionalCommand(
                        new InstantCommand().andThen(() ->
                                AutoBuilder.pathfindToPose(
                                        scoringPositionSupplier.get().getBluePose(),
                                        new PathConstraints(5.5, 5.5,
                                                Units.degreesToRadians(540d), Units.degreesToRadians(720d)), 0d).schedule()),
                        new InstantCommand().andThen(() ->
                                AutoBuilder.pathfindToPose(
                                        scoringPositionSupplier.get().getRedPose(),
                                        new PathConstraints(5.5, 5.5,
                                                Units.degreesToRadians(540d), Units.degreesToRadians(720d)), 0d).schedule()),
                        () -> DriverStation.getAlliance().isPresent()
                                && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)
                )
        );
    }

    public static Command driveToPose(ScoringPosition scoringPosition,
                                      double maxVelocity, double maxAcceleration, double goalEndVelocity) {
        return new SequentialCommandGroup(
                new InstantCommand(() ->
                        commandSwerveDrivetrain.applyRequest(() ->
                                drive.withVelocityX(0).
                                        withVelocityX(0).
                                        withRotationalRate(0)
                        )
                ),
                new ConditionalCommand(
                        AutoBuilder.pathfindToPose(
                                scoringPosition.getBluePose(),
                                new PathConstraints(maxVelocity, maxAcceleration,
                                        Units.degreesToRadians(540d), Units.degreesToRadians(720d)), goalEndVelocity),
                        AutoBuilder.pathfindToPose(
                                scoringPosition.getRedPose(),
                                new PathConstraints(maxVelocity, maxAcceleration,
                                        Units.degreesToRadians(540d), Units.degreesToRadians(720d)), goalEndVelocity),
                        () -> DriverStation.getAlliance().isPresent()
                                && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)
                )
        );
    }

    public static Command driveToPose(ScoringPosition scoringPosition, double maxVelocity, double maxAcceleration, double maxAngularVelocity, double maxAngularAcceleration) {
        return new SequentialCommandGroup(
                new InstantCommand(() ->
                        commandSwerveDrivetrain.applyRequest(() ->
                                drive.withVelocityX(0).
                                        withVelocityX(0).
                                        withRotationalRate(0)
                        )
                ),
                new ConditionalCommand(
                        AutoBuilder.pathfindToPose(
                                scoringPosition.getBluePose(),
                                new PathConstraints(maxVelocity, maxAcceleration,
                                        Units.degreesToRadians(maxAngularVelocity), Units.degreesToRadians(maxAngularAcceleration)), 0d),
                        AutoBuilder.pathfindToPose(
                                scoringPosition.getRedPose(),
                                new PathConstraints(maxVelocity, maxAcceleration,
                                        Units.degreesToRadians(maxAngularVelocity), Units.degreesToRadians(maxAngularAcceleration)), 0d),
                        () -> DriverStation.getAlliance().isPresent()
                                && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)
                )
        );
    }

    public static Command autoScoreWithUnwind(ScoringPosition scoringPosition) {
        return new SequentialCommandGroup(
                driveToPose(scoringPosition)
                        .alongWith(ScoreCommands.Climber.slightUnwindAuton()
                                .andThen(ScoreCommands.Arm.armStable())),
                new ConditionalCommand(
                        ScoreCommands.Drive.autoAlignRAuton(),
                        ScoreCommands.Drive.autoAlignLAuton(),
                        scoringPosition::isRightSide
                ).alongWith(ScoreCommands.Score.score()),
                ScoreCommands.Score.place()
                        .until(() -> !intakeSubsystem.hasCoral())
                        .withTimeout(2)
        );
    }

    public static Command autoScore() {
        return new SequentialCommandGroup(
                new InstantCommand(aprilTagSubsystem::resetAutoAlignData),
                new SelectCommand<>(
                        Map.ofEntries(
                                Map.entry(ScoringPosition.REEF_SIDE_A, driveToPose(ScoringPosition.REEF_SIDE_A)),
                                Map.entry(ScoringPosition.REEF_SIDE_B, driveToPose(ScoringPosition.REEF_SIDE_B)),
                                Map.entry(ScoringPosition.REEF_SIDE_C, driveToPose(ScoringPosition.REEF_SIDE_C)),
                                Map.entry(ScoringPosition.REEF_SIDE_D, driveToPose(ScoringPosition.REEF_SIDE_D)),
                                Map.entry(ScoringPosition.REEF_SIDE_E, driveToPose(ScoringPosition.REEF_SIDE_E)),
                                Map.entry(ScoringPosition.REEF_SIDE_F, driveToPose(ScoringPosition.REEF_SIDE_F)),
                                Map.entry(ScoringPosition.REEF_SIDE_G, driveToPose(ScoringPosition.REEF_SIDE_G)),
                                Map.entry(ScoringPosition.REEF_SIDE_H, driveToPose(ScoringPosition.REEF_SIDE_H)),
                                Map.entry(ScoringPosition.REEF_SIDE_I, driveToPose(ScoringPosition.REEF_SIDE_I)),
                                Map.entry(ScoringPosition.REEF_SIDE_J, driveToPose(ScoringPosition.REEF_SIDE_J)),
                                Map.entry(ScoringPosition.REEF_SIDE_K, driveToPose(ScoringPosition.REEF_SIDE_K)),
                                Map.entry(ScoringPosition.REEF_SIDE_L, driveToPose(ScoringPosition.REEF_SIDE_L))
                        ),
                        RobotContainer::getCurrentScoringPosition
                ).until(RobotContainer::isReadyToAlign)
                        .alongWith(ScoreCommands.Arm.armStable().onlyIf(() -> !armSubsystem.isCommandRunning())),
                new InstantCommand(() -> aprilTagSubsystem.setDoneAutoDriving(true)),
                new ConditionalCommand(
                        ScoreCommands.Drive.autoAlignRAuton(),
                        ScoreCommands.Drive.autoAlignLAuton(),
                        () -> RobotContainer.getCurrentScoringPosition().isRightSide()
                ).alongWith(ScoreCommands.Score.score()),
                ScoreCommands.Score.place()
                        .until(() -> !intakeSubsystem.hasCoral())
                        .withTimeout(2)
        );
    }

    public static Command autoScoreChoice() {
        return new SequentialCommandGroup(
                new InstantCommand(aprilTagSubsystem::resetAutoAlignData),
                new SelectCommand<>(
                        Map.ofEntries(
                                Map.entry(ScoringPosition.REEF_SIDE_A, driveToPoseChoice(ScoringPosition.REEF_SIDE_A)),
                                Map.entry(ScoringPosition.REEF_SIDE_B, driveToPoseChoice(ScoringPosition.REEF_SIDE_B)),
                                Map.entry(ScoringPosition.REEF_SIDE_C, driveToPoseChoice(ScoringPosition.REEF_SIDE_C)),
                                Map.entry(ScoringPosition.REEF_SIDE_D, driveToPoseChoice(ScoringPosition.REEF_SIDE_D)),
                                Map.entry(ScoringPosition.REEF_SIDE_E, driveToPoseChoice(ScoringPosition.REEF_SIDE_E)),
                                Map.entry(ScoringPosition.REEF_SIDE_F, driveToPoseChoice(ScoringPosition.REEF_SIDE_F)),
                                Map.entry(ScoringPosition.REEF_SIDE_G, driveToPoseChoice(ScoringPosition.REEF_SIDE_G)),
                                Map.entry(ScoringPosition.REEF_SIDE_H, driveToPoseChoice(ScoringPosition.REEF_SIDE_H)),
                                Map.entry(ScoringPosition.REEF_SIDE_I, driveToPoseChoice(ScoringPosition.REEF_SIDE_I)),
                                Map.entry(ScoringPosition.REEF_SIDE_J, driveToPoseChoice(ScoringPosition.REEF_SIDE_J)),
                                Map.entry(ScoringPosition.REEF_SIDE_K, driveToPoseChoice(ScoringPosition.REEF_SIDE_K)),
                                Map.entry(ScoringPosition.REEF_SIDE_L, driveToPoseChoice(ScoringPosition.REEF_SIDE_L))
                        ),
                        RobotContainer::getCurrentScoringPosition
                ).until(RobotContainer::isReadyToAlign)
                        .alongWith(ScoreCommands.Arm.armStable().onlyIf(() -> !armSubsystem.isCommandRunning())),
                new InstantCommand(() -> aprilTagSubsystem.setDoneAutoDriving(true)),
                new ConditionalCommand(
                        ScoreCommands.Drive.autoAlignRAuton(),
                        ScoreCommands.Drive.autoAlignLAuton(),
                        () -> RobotContainer.getCurrentScoringPosition().isRightSide()
                ).alongWith(ScoreCommands.Score.score()),
                ScoreCommands.Score.place()
                        .until(() -> !intakeSubsystem.hasCoral())
                        .withTimeout(2)
        );
    }

    public static Command autoScore(ScoringPosition scoringPosition, double maxVelocity, double maxAcceleration) {
        return new SequentialCommandGroup(
                driveToPose(scoringPosition, maxVelocity, maxAcceleration, 1d),
                new ConditionalCommand(
                        ScoreCommands.Drive.autoAlignRAuton(),
                        ScoreCommands.Drive.autoAlignLAuton(),
                        scoringPosition::isRightSide
                ).alongWith(ScoreCommands.Score.score()),
                ScoreCommands.Score.place()
                        .until(() -> !intakeSubsystem.hasCoral())
                        .withTimeout(2)
        );
    }

    public static Command autoScoreBlueBarge(ScoringPosition scoringPosition) {
        return new SequentialCommandGroup(
                driveToPose(scoringPosition, 4, 4, 0)
                        .alongWith(ScoreCommands.Arm.armBarge())
                        .alongWith(new InstantCommand(() -> intakeSubsystem.setIntakeMotors(120, 120))),
                ScoreCommands.Score.scoreBarge(),
                new InstantCommand(() -> intakeSubsystem.setIntakeMotors(-100, -100))
        );
    }

    //TODO: add the canceling auto drive early to autons and make sure tolerance is increased for auto align

    public static Command algaeBlue() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> commandSwerveDrivetrain
                        .resetPose(Constants.Vision.ALGAE_BLUE_POSE)),
                new InstantCommand(() -> RobotContainer.setState(State.L4)),
                autoScoreWithUnwind(ScoringPosition.REEF_SIDE_G),
                ScoreCommands.Drive.autoAlignCenterBackAuton().withTimeout(1d)
                        .alongWith(ScoreCommands.Score.removeAlgaeLow())
                        .alongWith(new InstantCommand(() -> intakeSubsystem.setIntakeMotors(120, 120))),
                ScoreCommands.Drive.autoAlignCenterAuton().withTimeout(1d)
                        .until(intakeSubsystem::hasAlgae),
                ScoreCommands.Drive.autoAlignCenterBackAuton().withTimeout(1d),
                new WaitCommand(.25),
                autoScoreBlueBarge(ScoringPosition.BARGE),
                driveToPose(ScoringPosition.REEF_SIDE_I)
                        .alongWith(ScoreCommands.Score.removeAlgaeHigh()),
                ScoreCommands.Drive.autoAlignCenterBackAuton().withTimeout(1d),
                ScoreCommands.Drive.autoAlignCenterAuton().withTimeout(1d)
                        .until(intakeSubsystem::hasAlgae)
                        .alongWith(new InstantCommand(() -> intakeSubsystem.setIntakeMotors(120, 120))),
                ScoreCommands.Drive.autoAlignCenterBackAuton().withTimeout(1d),
                new WaitCommand(.25),
                autoScoreBlueBarge(ScoringPosition.BARGE2),
                driveToPose(ScoringPosition.REEF_SIDE_EF)
                        .alongWith(ScoreCommands.Score.removeAlgaeHigh()),
                ScoreCommands.Drive.autoAlignCenterBackAuton()
        );
    }

    public static Command algaeRed() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> commandSwerveDrivetrain
                        .resetPose(Constants.Vision.ALGAE_RED_POSE)),
                new InstantCommand(() -> RobotContainer.setState(State.L4)),
                autoScoreWithUnwind(ScoringPosition.REEF_SIDE_G),
                ScoreCommands.Drive.autoAlignCenterBackAuton().withTimeout(1d)
                        .alongWith(ScoreCommands.Score.removeAlgaeLow())
                        .alongWith(new InstantCommand(() -> intakeSubsystem.setIntakeMotors(120, 120))),
                ScoreCommands.Drive.autoAlignCenterAuton()
                        .until(intakeSubsystem::hasAlgae)
                        .withTimeout(1d),
                ScoreCommands.Drive.autoAlignCenterBackAuton().withTimeout(1d),
                new WaitCommand(.25),
                autoScoreBlueBarge(ScoringPosition.BARGE),
                driveToPose(ScoringPosition.REEF_SIDE_I)
                        .alongWith(ScoreCommands.Score.removeAlgaeHigh()),
                ScoreCommands.Drive.autoAlignCenterBackAuton().withTimeout(1d),
                ScoreCommands.Drive.autoAlignCenterAuton().withTimeout(1d)
                        .until(intakeSubsystem::hasAlgae)
                        .alongWith(new InstantCommand(() -> intakeSubsystem.setIntakeMotors(120, 120))),
                ScoreCommands.Drive.autoAlignCenterBackAuton().withTimeout(1d),
                new WaitCommand(.25),
                autoScoreBlueBarge(ScoringPosition.BARGE2),
                driveToPose(ScoringPosition.REEF_SIDE_EF)
                        .alongWith(ScoreCommands.Score.removeAlgaeHigh()),
                ScoreCommands.Drive.autoAlignCenterBackAuton()
        );
    }

    public static Command threePieceFrontRightBlue() {
        return new SequentialCommandGroup(
                new PathPlannerAuto("3 Piece Front Right Blue Part 1"),
                new ConditionalCommand(
                        new PathPlannerAuto("3 Piece Front Right Blue Part 2"),
                        new PathPlannerAuto("3 Piece Front Right Blue Part 2 Fail"),
                        intakeSubsystem::hasCoral
                )
        );
    }

    public static Command threePieceFrontLeftBlue() {
        return new SequentialCommandGroup(
                new PathPlannerAuto("3 Piece Front Left Blue Part 1"),
                new ConditionalCommand(
                        new PathPlannerAuto("3 Piece Front Left Blue Part 2"),
                        new PathPlannerAuto("3 Piece Front Left Blue Part 2 Fail"),
                        intakeSubsystem::hasCoral
                )
        );
    }

    public static Command threePieceFrontRightRed() {
        return new SequentialCommandGroup(
                new PathPlannerAuto("3 Piece Front Right Red Part 1"),
                new ConditionalCommand(
                        new PathPlannerAuto("3 Piece Front Right Red Part 2"),
                        new PathPlannerAuto("3 Piece Front Right Red Part 2 Fail"),
                        intakeSubsystem::hasCoral
                )
        );
    }

    public static Command threePieceFrontLeftRed() {
        return new SequentialCommandGroup(
                new PathPlannerAuto("3 Piece Front Left Red Part 1"),
                new ConditionalCommand(
                        new PathPlannerAuto("3 Piece Front Left Red Part 2"),
                        new PathPlannerAuto("3 Piece Front Left Red Part 2 Fail"),
                        intakeSubsystem::hasCoral
                )
        );
    }
}
