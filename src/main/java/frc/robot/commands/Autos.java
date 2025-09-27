/// / Copyright (c) FIRST and other WPILib contributors.
/// / Open Source Software; you can modify and/or share it under the terms of
/// / the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.UserInterface;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.utility.ScoringPosition;
import frc.robot.utility.State;

import java.awt.*;
import java.util.Map;

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

    private static PathPlannerAuto fourPieceBlueBottomL4;

    private static PathPlannerAuto helper2PieceTopBlue;
    private static PathPlannerAuto helper2PieceBottomRed;

    private static final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    private static final WristSubsystem wristSubsystem = WristSubsystem.getInstance();
    private static final CommandSwerveDrivetrain commandSwerveDrivetrain = RobotContainer.commandSwerveDrivetrain;
    public final static SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDesaturateWheelSpeeds(true)
            .withDeadband(RobotContainer.MaxSpeed * .05).withRotationalDeadband(RobotContainer.MaxAngularRate * .05) // Add a 10% deadband
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);


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
        onePieceBlueDropL1 = new PathPlannerAuto("Blue Climber Drop");
        onePieceRedDropL1 = new PathPlannerAuto("Red Climber Drop");

        onePieceBlueRightL4 = new PathPlannerAuto("1 Piece Blue Right L4");
        onePieceBlueLeftL4 = new PathPlannerAuto("1 Piece Blue Left L4");
        onePieceRedRightL4 = new PathPlannerAuto("1 Piece Red Right L4");
        onePieceRedLeftL4 = new PathPlannerAuto("1 Piece Red Left L4");

        helper2PieceTopBlue = new PathPlannerAuto("2 Piece Blue Top L4 Helper");
        helper2PieceBottomRed = new PathPlannerAuto("2 Piece Red Bottom L4 Helper");

        twoPieceBlueBottomL4 = new PathPlannerAuto("2 Piece Blue Bottom L4");
        twoPieceBlueTopL4 = new PathPlannerAuto("2 Piece Blue Top L4");
        twoPieceRedBottomL4 = new PathPlannerAuto("2 Piece Red Bottom L4");
        twoPieceRedTopL4 = new PathPlannerAuto("2 Piece Red Top L4");

        threePieceBlueBottomL4 = new PathPlannerAuto("3 Piece Blue Bottom L4");
        threePieceBlueTopL4 = new PathPlannerAuto("3 Piece Blue Top L4");
        threePieceRedBottomL4 = new PathPlannerAuto("3 Piece Red Bottom L4");
        threePieceRedTopL4 = new PathPlannerAuto("3 Piece Red Top L4");

        fourPieceBlueBottomL4 = new PathPlannerAuto("4 Piece Blue Bottom L4");

        testBlue = new PathPlannerAuto("Test Blue");

        Shuffleboard.getTab("Autons").add("Red Autons", autonChooserRed)
                .withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0)
                .withSize(2, 1);
        Shuffleboard.getTab("Autons").add("Blue Autons", autonChooserBlue)
                .withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0)
                .withSize(2, 1);

        autonChooserRed.addOption("1 Piece Red Climber Drop", onePieceRedDropL1);
        autonChooserRed.addOption("1 Piece Red Right", onePieceRedRightL4);
        autonChooserRed.addOption("1 Piece Red Left", onePieceRedLeftL4);
        autonChooserRed.addOption("2 Piece Red Left", twoPieceRedBottomL4);
        autonChooserRed.addOption("2 Piece Red Right", twoPieceRedTopL4);
        autonChooserRed.addOption("3 Piece Red Left", threePieceRedBottomL4);
        autonChooserRed.addOption("3 Piece Red Right", threePieceRedTopL4);
        autonChooserRed.addOption("2 Piece Red Left Helper", helper2PieceBottomRed);

        autonChooserBlue.addOption("1 Piece Blue Climber Drop", onePieceBlueDropL1);
        autonChooserBlue.addOption("1 Piece Blue Right", onePieceBlueRightL4);
        autonChooserBlue.addOption("1 Piece Blue Left", onePieceBlueLeftL4);
        autonChooserBlue.addOption("2 Piece Blue Left", twoPieceBlueTopL4);
        autonChooserBlue.addOption("2 Piece Blue Right", twoPieceBlueBottomL4);
        autonChooserBlue.addOption("3 Piece Blue Left", threePieceBlueTopL4);
        autonChooserBlue.addOption("3 Piece Blue Right", threePieceBlueBottomL4);
        autonChooserBlue.addOption("2 Piece Blue Left Helper", helper2PieceTopBlue);
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
                                new PathConstraints(2d, 2d,
                                        Units.degreesToRadians(540d), Units.degreesToRadians(720d)), 0d),
                        AutoBuilder.pathfindToPose(
                                scoringPosition.getRedPose(),
                                new PathConstraints(2d, 2d,
                                        Units.degreesToRadians(540d), Units.degreesToRadians(720d)), 0d),
                        () -> DriverStation.getAlliance().isPresent()
                                && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)
                ),
                new ConditionalCommand(
                        AutoBuilder.pathfindToPose(
                                scoringPosition.getBluePose(),
                                new PathConstraints(2d, 2d,
                                        Units.degreesToRadians(540d), Units.degreesToRadians(720d)), 0d),
                        AutoBuilder.pathfindToPose(
                                scoringPosition.getRedPose(),
                                new PathConstraints(2d, 2d,
                                        Units.degreesToRadians(540d), Units.degreesToRadians(720d)), 0d),
                        () -> DriverStation.getAlliance().isPresent()
                                && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)
                )
        );
    }

    public static Command driveToPose(ScoringPosition scoringPosition, double maxVelocity, double maxAcceleration) {
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
                                        Units.degreesToRadians(540d), Units.degreesToRadians(720d)), 0d),
                        AutoBuilder.pathfindToPose(
                                scoringPosition.getRedPose(),
                                new PathConstraints(maxVelocity, maxAcceleration,
                                        Units.degreesToRadians(540d), Units.degreesToRadians(720d)), 0d),
                        () -> DriverStation.getAlliance().isPresent()
                                && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)
                ),
                new ConditionalCommand(
                        AutoBuilder.pathfindToPose(
                                scoringPosition.getBluePose(),
                                new PathConstraints(maxVelocity, maxAcceleration,
                                        Units.degreesToRadians(540d), Units.degreesToRadians(720d)), 0d),
                        AutoBuilder.pathfindToPose(
                                scoringPosition.getRedPose(),
                                new PathConstraints(maxVelocity, maxAcceleration,
                                        Units.degreesToRadians(540d), Units.degreesToRadians(720d)), 0d),
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

    public static Command autoScore(ScoringPosition scoringPosition) {
        return new SequentialCommandGroup(
                driveToPose(scoringPosition).alongWith(ScoreCommands.Arm.armStable()),
                new ConditionalCommand(
                        ScoreCommands.Drive.autoAlignRAuton(),
                        ScoreCommands.Drive.autoAlignLAuton(),
                        scoringPosition::isRightSide
                ).alongWith(ScoreCommands.Score.score()),
                ScoreCommands.Score.place()
                        .until(() -> intakeSubsystem.isAboveSpeed() && !intakeSubsystem.hasCoral())
                        .withTimeout(2),
                ScoreCommands.Intake.intakeHP()
        );
    }

    public static Command autoScore() {
        return new SequentialCommandGroup(
                new SelectCommand<>(
                        Map.ofEntries(
                                Map.entry(ScoringPosition.REEF_SIDE_A, driveToPose(ScoringPosition.REEF_SIDE_A, 5d, 5d)),
                                Map.entry(ScoringPosition.REEF_SIDE_B, driveToPose(ScoringPosition.REEF_SIDE_B, 5d, 5d)),
                                Map.entry(ScoringPosition.REEF_SIDE_C, driveToPose(ScoringPosition.REEF_SIDE_C, 5d, 5d)),
                                Map.entry(ScoringPosition.REEF_SIDE_D, driveToPose(ScoringPosition.REEF_SIDE_D, 5d, 5d)),
                                Map.entry(ScoringPosition.REEF_SIDE_E, driveToPose(ScoringPosition.REEF_SIDE_E, 5d, 5d)),
                                Map.entry(ScoringPosition.REEF_SIDE_F, driveToPose(ScoringPosition.REEF_SIDE_F, 5d, 5d)),
                                Map.entry(ScoringPosition.REEF_SIDE_G, driveToPose(ScoringPosition.REEF_SIDE_G, 5d, 5d)),
                                Map.entry(ScoringPosition.REEF_SIDE_H, driveToPose(ScoringPosition.REEF_SIDE_H, 5d, 5d)),
                                Map.entry(ScoringPosition.REEF_SIDE_I, driveToPose(ScoringPosition.REEF_SIDE_I, 5d, 5d)),
                                Map.entry(ScoringPosition.REEF_SIDE_J, driveToPose(ScoringPosition.REEF_SIDE_J, 5d, 5d)),
                                Map.entry(ScoringPosition.REEF_SIDE_K, driveToPose(ScoringPosition.REEF_SIDE_K, 5d, 5d)),
                                Map.entry(ScoringPosition.REEF_SIDE_L, driveToPose(ScoringPosition.REEF_SIDE_L, 5d, 5d))
                        ),
                        RobotContainer::getCurrentScoringPosition
                ),
                new ConditionalCommand(
                        ScoreCommands.Drive.autoAlignRAuton(),
                        ScoreCommands.Drive.autoAlignLAuton(),
                        () -> RobotContainer.getCurrentScoringPosition().isRightSide()
                ).alongWith(ScoreCommands.Score.score()),
                ScoreCommands.Score.place()
                        .until(() -> intakeSubsystem.isAboveSpeed() && !intakeSubsystem.hasCoral())
                        .withTimeout(2),
                ScoreCommands.Intake.intakeHP()
        );
    }

    public static Command autoScore(ScoringPosition scoringPosition, double maxVelocity, double maxAcceleration) {
        return new SequentialCommandGroup(
                driveToPose(scoringPosition, maxVelocity, maxAcceleration).alongWith(ScoreCommands.Arm.armStable()),
                new ConditionalCommand(
                        ScoreCommands.Drive.autoAlignRAuton(),
                        ScoreCommands.Drive.autoAlignLAuton(),
                        scoringPosition::isRightSide
                ).alongWith(ScoreCommands.Score.score()),
                ScoreCommands.Score.place()
                        .until(() -> intakeSubsystem.isAboveSpeed() && !intakeSubsystem.hasCoral())
                        .withTimeout(2),
                ScoreCommands.Intake.intakeHP()
        );
    }
}
