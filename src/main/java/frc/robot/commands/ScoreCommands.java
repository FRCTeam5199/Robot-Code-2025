package frc.robot.commands;

import java.util.Map;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants.ArmConstants;
import frc.robot.constants.Constants.ElevatorConstants;
import frc.robot.constants.Constants.WristConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.template.PositionCommand;
import frc.robot.subsystems.template.VelocityCommand;
import frc.robot.utility.State;

import static frc.robot.RobotContainer.*;

public class ScoreCommands {
    private static ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private static ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();
    private static WristSubsystem wristSubsystem = WristSubsystem.getInstance();
    private static IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    private static ClimberSubsystem climberSubsystem = ClimberSubsystem.getInstance();

    private static AprilTagSubsystem aprilTagSubsystem = AprilTagSubsystem.getInstance();

    private static Timer timer = new Timer();

    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.baseUnitMagnitude();

    private final static SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDesaturateWheelSpeeds(true) // Add a 10% deadband
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);
    public final static SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric().withDesaturateWheelSpeeds(true)
            .withDeadband(MaxSpeed * .05).withRotationalDeadband(MaxAngularRate * .05) // Add a 10% deadband
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);
    public static final PIDController turnToPiecePIdController2 = new PIDController(.1, 0.0, 0.02);
    private static double goalRotation = 0;

    public static class Drive {
        public static Command autoAlignTeleop() {
            return new ConditionalCommand(
                    new FunctionalCommand(
                            () -> commandSwerveDrivetrain.resetPose(
                                    new Pose2d(
                                            new Translation2d(commandSwerveDrivetrain.getPose().getX(),
                                                    commandSwerveDrivetrain.getPose().getY()),
                                            new Rotation2d(Math.toRadians(aprilTagSubsystem.getRotationToAlign(aprilTagSubsystem
                                                    .getClosestTagID()))))),
                            () -> {
                                commandSwerveDrivetrain.setControl(
                                        drive.withVelocityX(xVelocity)
                                                .withVelocityY(yVelocity)
                                                .withRotationalRate(rotationVelocity));
                            },
                            (interrupted) -> {
                                commandSwerveDrivetrain.resetPose(
                                        new Pose2d(
                                                new Translation2d(commandSwerveDrivetrain.getPose().getX(),
                                                        commandSwerveDrivetrain.getPose().getY()),
                                                new Rotation2d(Math.toRadians(commandSwerveDrivetrain
                                                        .getPigeon2().getRotation2d().getDegrees() + (DriverStation.getAlliance().isPresent()
                                                        && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue) ? 0 : 180)))));
                                commandSwerveDrivetrain.setControl(
                                        drive.withVelocityX(0)
                                                .withVelocityY(0)
                                                .withRotationalRate(0));
                            },
                            () -> (RobotContainer.aligned() && commandSwerveDrivetrain.getState().Speeds.vxMetersPerSecond < .01
                                    && commandSwerveDrivetrain.getState().Speeds.vyMetersPerSecond < .01),
                            commandSwerveDrivetrain
                    ),
                    commandSwerveDrivetrain.applyRequest(() -> drive
                            .withVelocityX(-commandXboxController.getLeftY() * MaxSpeed)
                            .withVelocityY(-commandXboxController.getLeftX() * MaxSpeed)
                            .withRotationalRate(-commandXboxController.getRightX() * MaxAngularRate)),
                    () -> RobotContainer.isUseAutoAlign()
                            && (RobotContainer.getState() == State.L2
                            || RobotContainer.getState() == State.L3
                            || RobotContainer.getState() == State.L4));
        }

        public static Command autoAlignCenterAuton() {
            return new FunctionalCommand(
                    () -> {
                        RobotContainer.setAutoAlignOffsetCenter();

                        commandSwerveDrivetrain.resetPose(
                                new Pose2d(
                                        new Translation2d(commandSwerveDrivetrain.getPose().getX(),
                                                commandSwerveDrivetrain.getPose().getY()),
                                        new Rotation2d(Math.toRadians(aprilTagSubsystem.getRotationToAlign(aprilTagSubsystem
                                                .getClosestTagID())))));
                    },
                    () ->
                            RobotContainer.commandSwerveDrivetrain.setControl(
                                    drive.withVelocityX(RobotContainer.xVelocity)
                                            .withVelocityY(yVelocity)
                                            .withRotationalRate(rotationVelocity)),
                    (interrupted) -> {
                        RobotContainer.commandSwerveDrivetrain
                                .resetRotation(new Rotation2d(Math.toRadians(RobotContainer.commandSwerveDrivetrain
                                        .getPigeon2().getRotation2d().getDegrees() +
                                        (DriverStation.getAlliance().isPresent()
                                                && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)
                                                ? 0 : 180))));

                        RobotContainer.commandSwerveDrivetrain.setControl(
                                drive.withVelocityX(0)
                                        .withVelocityY(0)
                                        .withRotationalRate(0));
                    },
                    RobotContainer::aligned,
                    RobotContainer.commandSwerveDrivetrain
            ).onlyIf(RobotContainer::isUseAutoAlign);
        }

        public static Command autoAlignCenterBackAuton() {
            return new FunctionalCommand(
                    () -> {
                        RobotContainer.setAutoAlignOffsetCenterBack();

                        commandSwerveDrivetrain.resetPose(
                                new Pose2d(
                                        new Translation2d(commandSwerveDrivetrain.getPose().getX(),
                                                commandSwerveDrivetrain.getPose().getY()),
                                        new Rotation2d(Math.toRadians(aprilTagSubsystem.getRotationToAlign(aprilTagSubsystem
                                                .getClosestTagID())))));
                    },
                    () ->
                            RobotContainer.commandSwerveDrivetrain.setControl(
                                    drive.withVelocityX(RobotContainer.xVelocity)
                                            .withVelocityY(yVelocity)
                                            .withRotationalRate(rotationVelocity)),
                    (interrupted) -> {
                        RobotContainer.commandSwerveDrivetrain
                                .resetRotation(new Rotation2d(Math.toRadians(RobotContainer.commandSwerveDrivetrain
                                        .getPigeon2().getRotation2d().getDegrees() +
                                        (DriverStation.getAlliance().isPresent()
                                                && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)
                                                ? 0 : 180))));

                        RobotContainer.commandSwerveDrivetrain.setControl(
                                drive.withVelocityX(0)
                                        .withVelocityY(0)
                                        .withRotationalRate(0));
                    },
                    RobotContainer::aligned,
                    RobotContainer.commandSwerveDrivetrain
            ).onlyIf(RobotContainer::isUseAutoAlign);
        }

        public static Command autoAlignLAuton() {
            return new FunctionalCommand(
                    () -> {
                        RobotContainer.setAutoAlignOffsetLeft();

                        commandSwerveDrivetrain.resetPose(
                                new Pose2d(
                                        new Translation2d(commandSwerveDrivetrain.getPose().getX(),
                                                commandSwerveDrivetrain.getPose().getY()),
                                        new Rotation2d(Math.toRadians(aprilTagSubsystem.getRotationToAlign(aprilTagSubsystem
                                                .getClosestTagID())))));
                    },
                    () -> RobotContainer.commandSwerveDrivetrain.setControl(
                            drive.withVelocityX(RobotContainer.xVelocity)
                                    .withVelocityY(yVelocity)
                                    .withRotationalRate(rotationVelocity)),
                    (interrupted) -> {
                        commandSwerveDrivetrain.resetPose(
                                new Pose2d(
                                        new Translation2d(commandSwerveDrivetrain.getPose().getX(),
                                                commandSwerveDrivetrain.getPose().getY()),
                                        new Rotation2d(Math.toRadians(RobotContainer.commandSwerveDrivetrain
                                                .getPigeon2().getRotation2d().getDegrees() +
                                                (DriverStation.getAlliance().isPresent()
                                                        && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)
                                                        ? 0 : 180)))));

                        RobotContainer.commandSwerveDrivetrain.setControl(
                                drive.withVelocityX(0)
                                        .withVelocityY(0)
                                        .withRotationalRate(0));
                    },
                    RobotContainer::aligned,
                    RobotContainer.commandSwerveDrivetrain
            ).onlyIf(RobotContainer::isUseAutoAlign);
        }

        public static Command autoAlignRAuton() {
            return new FunctionalCommand(
                    () -> {
                        RobotContainer.setAutoAlignOffsetRight();

                        commandSwerveDrivetrain.resetPose(
                                new Pose2d(
                                        new Translation2d(commandSwerveDrivetrain.getPose().getX(),
                                                commandSwerveDrivetrain.getPose().getY()),
                                        new Rotation2d(Math.toRadians(aprilTagSubsystem.getRotationToAlign(aprilTagSubsystem
                                                .getClosestTagID())))));
                    },
                    () ->
                            RobotContainer.commandSwerveDrivetrain.setControl(
                                    drive.withVelocityX(RobotContainer.xVelocity)
                                            .withVelocityY(yVelocity)
                                            .withRotationalRate(rotationVelocity)),
                    (interrupted) -> {
                        RobotContainer.commandSwerveDrivetrain
                                .resetRotation(new Rotation2d(Math.toRadians(RobotContainer.commandSwerveDrivetrain
                                        .getPigeon2().getRotation2d().getDegrees() +
                                        (DriverStation.getAlliance().isPresent()
                                                && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)
                                                ? 0 : 180))));

                        RobotContainer.commandSwerveDrivetrain.setControl(
                                drive.withVelocityX(0)
                                        .withVelocityY(0)
                                        .withRotationalRate(0));
                    },
                    RobotContainer::aligned,
                    RobotContainer.commandSwerveDrivetrain
            ).onlyIf(RobotContainer::isUseAutoAlign);
        }

        //        public static Command driveToPiece() {
//            return new FunctionalCommand(
//                    () -> {
//                    },
//                    () -> {
//                        commandSwerveDrivetrain.setControl(robotCentricDrive.withVelocityX(1.2)
//                                .withRotationalRate(driveToPieceRotationVelocity));
//                    },
//                    (interrupted) -> {
//                        commandSwerveDrivetrain.setControl(drive);
//                    },
//                    () -> (false),
//                    commandSwerveDrivetrain
//            );
//        }
//
        public static Command driveForward() {
            return new FunctionalCommand(
                    () -> {
                    },
                    () -> {
                        commandSwerveDrivetrain.setControl(robotCentricDrive.withVelocityX(1.25));
                    },
                    (interrupted) -> {
                        commandSwerveDrivetrain.setControl(drive);
                    },
                    () -> (false),
                    commandSwerveDrivetrain
            );
        }
//
//        public static Command driveForward(double goalRotation) {
//            return new FunctionalCommand(
//                    () -> {
//                    },
//                    () -> {
//                        double rotationRate = turnToPiecePIdController2
//                                .calculate(commandSwerveDrivetrain.getPose().getRotation().getDegrees(), goalRotation);
//                        commandSwerveDrivetrain.setControl(robotCentricDrive.withVelocityX(1)
//                                .withRotationalRate(rotationRate));
//                    },
//                    (interrupted) -> {
//                        commandSwerveDrivetrain.setControl(drive);
//                    },
//                    () -> (false),
//                    commandSwerveDrivetrain
//            );
//        }

        public static Command autoMoveForwardBottom() {
            return new FunctionalCommand(
                    () -> {
                    },
                    () -> {
                        if (DriverStation.getAlliance().isPresent()
                                && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue))
                            RobotContainer.commandSwerveDrivetrain.setControl(
                                    drive.withVelocityX(-.75)
                                            .withVelocityY(-.75));
                        else RobotContainer.commandSwerveDrivetrain.setControl(
                                drive.withVelocityX(-.75)
                                        .withVelocityY(.75));
                    },
                    (interrupted) -> RobotContainer.commandSwerveDrivetrain.setControl(
                            drive.withVelocityX(0)
                                    .withVelocityY(0)),
                    () -> false,
                    RobotContainer.commandSwerveDrivetrain);
        }


        public static Command autoMoveForwardTop() {
            return new FunctionalCommand(
                    () -> {
                    },
                    () -> {
                        if (DriverStation.getAlliance().isPresent()
                                && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue))
                            RobotContainer.commandSwerveDrivetrain.setControl(
                                    drive.withVelocityX(-0.75)
                                            .withVelocityY(0.75));
                        else RobotContainer.commandSwerveDrivetrain.setControl(
                                drive.withVelocityX(-0.75)
                                        .withVelocityY(-0.75));
                    },
                    (interrupted) -> {
                        RobotContainer.commandSwerveDrivetrain.setControl(
                                drive.withVelocityX(0)
                                        .withVelocityY(0));
                    },
                    () -> false,
                    RobotContainer.commandSwerveDrivetrain);
        }

    }

    public static class Stabling {
        public static Command regularStable() {
            return new SequentialCommandGroup(
                    new PositionCommand(wristSubsystem, WristConstants.L4_PREP)
                            .onlyIf(() -> RobotContainer.getState() == State.L4),
                    new PositionCommand(elevatorSubsystem, ElevatorConstants.STABLE, 80, 120)
                            .until(() -> elevatorSubsystem.isAtBottom()
                                    && elevatorSubsystem.getMechM() < .05),
//                    new InstantCommand(() -> elevatorSubsystem.getMotor().setPosition(0)),
                    Arm.armStable()
            );
        }

        public static Command bargeStable() {
            return new SequentialCommandGroup(
                    new ParallelCommandGroup(
                            new PositionCommand(elevatorSubsystem, ElevatorConstants.STABLE, 80, 120)
                                    .until(() -> elevatorSubsystem.isAtBottom()
                                            && elevatorSubsystem.getMechM() < .05),
                            new PositionCommand(wristSubsystem, WristConstants.STABLE, 50, 50)
                    ),
//                    new InstantCommand(() -> elevatorSubsystem.getMotor().setPosition(0)),
                    Arm.armStable()
            );
        }

        public static Command groundIntakeStable() {
            return new SequentialCommandGroup(
                    new ParallelCommandGroup(
                            new PositionCommand(elevatorSubsystem, 0.02, 80, 120)
                                    .until(() -> elevatorSubsystem.isAtBottom()
                                            && elevatorSubsystem.getMechM() < .05),
                            new PositionCommand(wristSubsystem, WristConstants.STABLE)
                    ),
//                    new InstantCommand(() -> elevatorSubsystem.getMotor().setPosition(0)),
                    Arm.armStable()
            )/*.alongWith(Intake.groundIntakeSequence())*/.alongWith(new InstantCommand(() -> intakeSubsystem.setPercent(0)));
        }

        public static Command stableL4() {
            return new SequentialCommandGroup(
                    new PositionCommand(wristSubsystem, WristConstants.HP),
                    new ParallelCommandGroup(
                            new PositionCommand(elevatorSubsystem, ElevatorConstants.STABLE, 80, 120)
                                    .until(() -> elevatorSubsystem.isAtBottom()
                                            && elevatorSubsystem.getMechM() < .05),
                            new PositionCommand(wristSubsystem, WristConstants.HP)
                    ),
//                    new InstantCommand(() -> elevatorSubsystem.getMotor().setPosition(0)),
                    Arm.armStable()
            );
        }

        public static Command algaeStable() {
            return new SequentialCommandGroup(
                    new ParallelCommandGroup(
                            new PositionCommand(elevatorSubsystem, ElevatorConstants.ALGAE_STABLE, 80, 120),
                            new PositionCommand(wristSubsystem, WristConstants.ALGAE_STABLE)
                    ),
                    Arm.armStable()
            );
        }

        public static Command groundAlgaeStable() {
            return new SequentialCommandGroup(
                    Arm.armStable(),
                    new ParallelCommandGroup(
                            new PositionCommand(elevatorSubsystem, ElevatorConstants.ALGAE_STABLE, 80, 120),
                            new PositionCommand(wristSubsystem, WristConstants.BARGE)
                    )
            );
        }

        public static Command stable() {
            return new ConditionalCommand(
                    algaeStable(),
                    new ConditionalCommand(
                            bargeStable(),
                            regularStable(),
                            () -> RobotContainer.getState() == State.BARGE
                    ),
                    () -> (RobotContainer.getState() == State.ALGAE_LOW
                            || RobotContainer.getState() == State.ALGAE_HIGH)
            );
        }

        public static Command intakeStable() {
            return new SequentialCommandGroup(
                    new PositionCommand(elevatorSubsystem, ElevatorConstants.STABLE, 80, 120)
                            .andThen(new PositionCommand(armSubsystem, ArmConstants.HP)),
                    new PositionCommand(wristSubsystem, WristConstants.HP)
            );
        }

        public static Command wristandElevatorStable() {
            return new PositionCommand(elevatorSubsystem, ElevatorConstants.STABLE, 80, 120)
                    .alongWith(new PositionCommand(wristSubsystem, WristConstants.STABLE));
        }
    }

    public static class Climber {
        public static Command drop() {
            return new FunctionalCommand(
                    () -> {
                        timer.reset();
                    },
                    () -> climberSubsystem.drop(),
                    (bool) -> climberSubsystem.stopDrop(),
                    () -> {
                        timer.start();
                        if (timer.hasElapsed(0.45)) {
                            timer.stop();
                            return true;
                        }
                        return false;
                    },
                    climberSubsystem);
        }

        public static Command slightUnwindAuton() {
            return new InstantCommand(() -> climberSubsystem.setPercent(-1))
                    .andThen(new WaitCommand(.4)).andThen(() -> climberSubsystem.setPercent(0));
        }
    }

    public static class Zeroing {
        public static Command zeroElevator() {
            return new FunctionalCommand(
                    () -> {
                        elevatorSubsystem.setVoltage(-2);
                        elevatorSubsystem.setOffset(0, false);
                    },
                    () -> {
                    },
                    (interrupted) -> elevatorSubsystem.setVoltage(0),
                    () -> (elevatorSubsystem.isAtBottom() && elevatorSubsystem.getMechM() < .05),
                    elevatorSubsystem
            );
        }

        public static Command zeroWrist() {
            return new FunctionalCommand(
                    () -> {
                        wristSubsystem.setPosition(10);
                        wristSubsystem.setVoltage(-1);
                        wristSubsystem.setOffset(0, false);
                    },
                    () -> {
                    },
                    (interrupted) -> wristSubsystem.setVoltage(0),
                    wristSubsystem::isAtBottom,
                    wristSubsystem
            );
        }


        public static Command zeroArm() {
            return new FunctionalCommand(
                    () -> {
                        armSubsystem.setVoltage(-5);
                        armSubsystem.setOffset(0, false);
                    },
                    () -> {
                    },
                    (interrupted) -> {
                        armSubsystem.setVelocity(0);
                    },
                    armSubsystem::isAtBottom,
                    armSubsystem
            );
        }

        public static Command zeroSubsystems() {
            return new SequentialCommandGroup(
                    new PositionCommand(wristSubsystem, WristConstants.STABLE),
                    new ParallelCommandGroup(
                            new InstantCommand(() -> elevatorSubsystem.setFollowLastMechProfile(false)),
                            new InstantCommand(() -> armSubsystem.setFollowLastMechProfile(false)),
                            new InstantCommand(() -> wristSubsystem.setFollowLastMechProfile(false))
                    ),
                    new ParallelCommandGroup(
                            zeroElevator(),
                            zeroArm()
                    ).withTimeout(5),
                    zeroWrist().withTimeout(2),
                    new WaitCommand(.25),
                    new ParallelCommandGroup(
                            new InstantCommand(() -> elevatorSubsystem.getMotor().setPosition(0)),
                            new InstantCommand(() -> armSubsystem.getMotor().setPosition(0)),
                            new InstantCommand(() -> wristSubsystem.getMotor().setPosition(0))
                    ),
                    new ParallelCommandGroup(
                            new InstantCommand(() -> elevatorSubsystem.setOffset(0)),
                            new InstantCommand(() -> armSubsystem.setOffset(0)),
                            new InstantCommand(() -> wristSubsystem.setOffset(0))
                    ),
                    new PositionCommand(wristSubsystem, WristConstants.STABLE)
//                    new ParallelCommandGroup(
//                            new PositionCommand(elevatorSubsystem, .2, true),
//                            new PositionCommand(armSubsystem, 7),
//                            new PositionCommand(wristSubsystem, 7)
//                    ),
//                    new ParallelCommandGroup(
//                            new PositionCommand(elevatorSubsystem, 0, false),
//                            new PositionCommand(armSubsystem, 0),
//                            new PositionCommand(wristSubsystem, 0)
//                    )
            );
        }
    }

    public static class Intake {
        public static Command intakeGround() {
            return new ConditionalCommand(
                    new ParallelCommandGroup(
                            new PositionCommand(armSubsystem, ArmConstants.GROUND),
                            new PositionCommand(elevatorSubsystem, ElevatorConstants.GROUND),
                            new PositionCommand(wristSubsystem, WristConstants.GROUND)
                    ),
                    new SequentialCommandGroup(
                            new PositionCommand(elevatorSubsystem, ElevatorConstants.GROUND, 80, 120),
                            new ParallelCommandGroup(
                                    new PositionCommand(armSubsystem, ArmConstants.GROUND),
                                    new PositionCommand(wristSubsystem, WristConstants.GROUND)
                            )
                    ),
                    () -> elevatorSubsystem.getMechM() < ElevatorConstants.GROUND
            );
        }

        public static Command intakeGroundPrep() {
            return new ConditionalCommand(
                    new ParallelCommandGroup(
                            new PositionCommand(armSubsystem, ArmConstants.GROUND),
                            new PositionCommand(elevatorSubsystem, ElevatorConstants.GROUND),
                            new PositionCommand(wristSubsystem, 100)
                    ),
                    new SequentialCommandGroup(
                            new PositionCommand(elevatorSubsystem, ElevatorConstants.GROUND, 80, 120),
                            new PositionCommand(armSubsystem, ArmConstants.GROUND),
                            new PositionCommand(wristSubsystem, 100)

                    ),
                    () -> elevatorSubsystem.getMechM() < ElevatorConstants.GROUND
            );
        }

        public static Command intakeHP() {
            return new ConditionalCommand(
                    new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new PositionCommand(elevatorSubsystem, ElevatorConstants.HP, 80, 120)
                                            .until(() -> elevatorSubsystem.getMechM() < .4)
                                            .andThen(new PositionCommand(armSubsystem, ArmConstants.HP)),
                                    new PositionCommand(wristSubsystem, WristConstants.HP)
                            )
                    ),
                    new SequentialCommandGroup(
                            new ParallelCommandGroup( //Going up
                                    new PositionCommand(elevatorSubsystem, ElevatorConstants.HP, 80, 120),
                                    new PositionCommand(armSubsystem, ArmConstants.HP),
                                    new PositionCommand(wristSubsystem, WristConstants.HP)
                            )
                    ),
                    () -> elevatorSubsystem.getMechM() > ElevatorConstants.HP
            );
        }

        public static Command intakeHPCoralStuck() {
            return new ConditionalCommand(
                    new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new PositionCommand(elevatorSubsystem, ElevatorConstants.HP, 80, 120)
                                            .until(() -> elevatorSubsystem.getMechM() < .3)
                                            .andThen(new PositionCommand(armSubsystem, ArmConstants.HP_C)),
                                    new PositionCommand(wristSubsystem, WristConstants.HP_C)
                            )
                    ),
                    new SequentialCommandGroup(
                            new ParallelCommandGroup( //Going up
                                    new PositionCommand(elevatorSubsystem, ElevatorConstants.HP),
                                    new PositionCommand(armSubsystem, ArmConstants.HP_C),
                                    new PositionCommand(wristSubsystem, WristConstants.HP_C)
                            )
                    ),
                    () -> elevatorSubsystem.getMechM() > ElevatorConstants.HP
            );
        }

        public static Command intakeGroundAlgae() {
            return new ConditionalCommand(
                    new ParallelCommandGroup(
                            new PositionCommand(armSubsystem, ArmConstants.ALGAE_GROUND),
                            new PositionCommand(elevatorSubsystem, ElevatorConstants.ALGAE_GROUND),
                            new PositionCommand(wristSubsystem, WristConstants.ALGAE_GROUND)
                    ),
                    new SequentialCommandGroup(
                            new PositionCommand(elevatorSubsystem, ElevatorConstants.ALGAE_GROUND, 80, 120),
                            new ParallelCommandGroup(
                                    new PositionCommand(armSubsystem, ArmConstants.ALGAE_GROUND),
                                    new PositionCommand(wristSubsystem, WristConstants.ALGAE_GROUND)
                            )
                    ),
                    () -> elevatorSubsystem.getMechM() < ElevatorConstants.ALGAE_GROUND
            ).alongWith(new VelocityCommand(intakeSubsystem, 120, 120));
        }
    }


    public static class Arm {
        public static Command armGroundIntake() {
            return new PositionCommand(armSubsystem, ArmConstants.GROUND);
        }

        public static Command armL1() {
            return new PositionCommand(armSubsystem, ArmConstants.L1)
                    .alongWith(new InstantCommand(() -> RobotContainer.setState(State.L1)))
//                    .alongWith(new InstantCommand(() -> intakeSubsystem.setPercent(0)))
                    .alongWith(new PositionCommand(wristSubsystem, WristConstants.L1))
                    .andThen(new PositionCommand(elevatorSubsystem, ElevatorConstants.STABLE));
        }

        public static Command armL2() {
            return new PositionCommand(armSubsystem, ArmConstants.L2)
                    .alongWith(new InstantCommand(() -> RobotContainer.setState(State.L2)))
//                    .alongWith(new InstantCommand(() -> intakeSubsystem.setPercent(0)))
                    .alongWith(new PositionCommand(wristSubsystem, WristConstants.L2))
                    .andThen(new PositionCommand(elevatorSubsystem, ElevatorConstants.STABLE));
        }

        public static Command armL3() {
            return new PositionCommand(armSubsystem, ArmConstants.L3)
                    .alongWith(new InstantCommand(() -> RobotContainer.setState(State.L3)))
//                    .alongWith(new InstantCommand(() -> intakeSubsystem.setPercent(0)))
                    .alongWith(new PositionCommand(wristSubsystem, WristConstants.L3))
                    .andThen(new PositionCommand(elevatorSubsystem, ElevatorConstants.STABLE));
        }

        public static Command armL4() {
            return new PositionCommand(armSubsystem, ArmConstants.L4)
                    .alongWith(new InstantCommand(() -> RobotContainer.setState(State.L4)))
//                    .alongWith(new InstantCommand(() -> intakeSubsystem.setPercent(0)))
                    .alongWith(new PositionCommand(wristSubsystem, WristConstants.L4))
                    .andThen(new PositionCommand(elevatorSubsystem, ElevatorConstants.STABLE));
        }

        public static Command armAlgaeLow() {
            return new PositionCommand(armSubsystem, ArmConstants.ALGAE_LOW)
                    .alongWith(new PositionCommand(wristSubsystem, WristConstants.ALGAE_PREP))
                    .alongWith(new InstantCommand(() -> RobotContainer.setState(State.ALGAE_LOW)));
        }

        public static Command armAlgaeHigh() {
            return new PositionCommand(armSubsystem, ArmConstants.ALGAE_HIGH)
                    .alongWith(new PositionCommand(wristSubsystem, WristConstants.ALGAE_PREP))
                    .alongWith(new InstantCommand(() -> RobotContainer.setState(State.ALGAE_HIGH)));
        }

        public static Command armBarge() {
            return new PositionCommand(armSubsystem, ArmConstants.BARGE, 80, 120)
                    .alongWith(new InstantCommand(() -> RobotContainer.setState(State.BARGE)));
        }

        public static Command armProcessor() {
            return new PositionCommand(armSubsystem, ArmConstants.PROCESSOR)
                    .alongWith(new InstantCommand(() -> RobotContainer.setState(State.PROCESSOR)));
        }

        public static Command armStable() {
            return new SelectCommand<>(
                    Map.ofEntries(
                            Map.entry(State.L1, armL1()),
                            Map.entry(State.L2, armL2()),
                            Map.entry(State.L3, armL3()),
                            Map.entry(State.L4, armL4()),
                            Map.entry(State.ALGAE_LOW, armAlgaeLow()),
                            Map.entry(State.ALGAE_HIGH, armAlgaeHigh()),
                            Map.entry(State.BARGE, armBarge()),
                            Map.entry(State.PROCESSOR, armProcessor())
                    ),
                    RobotContainer::getState
            );
        }
    }

    public static class Score {
        public static Command removeAlgaeHigh() {
            return new ConditionalCommand(
                    new SequentialCommandGroup( //Going down
                            new ParallelCommandGroup(
                                    new PositionCommand(elevatorSubsystem, ElevatorConstants.ALGAE_HIGH, 80, 120),
                                    new PositionCommand(wristSubsystem, WristConstants.ALGAE_HIGH)
                            ),
                            new PositionCommand(armSubsystem, ArmConstants.ALGAE_HIGH)
                    ),
                    new SequentialCommandGroup( //Going up
                            new PositionCommand(armSubsystem, ArmConstants.ALGAE_HIGH),
                            new ParallelCommandGroup(
                                    new PositionCommand(elevatorSubsystem, ElevatorConstants.ALGAE_HIGH),
                                    new PositionCommand(wristSubsystem, WristConstants.ALGAE_HIGH)
                            )
                    ),
                    () -> elevatorSubsystem.getMechM() > ElevatorConstants.ALGAE_HIGH
            );
        }

        public static Command removeAlgaeLow() {
            return new ConditionalCommand(
                    new SequentialCommandGroup( //Going down
                            new ParallelCommandGroup(
                                    new PositionCommand(elevatorSubsystem, ElevatorConstants.ALGAE_LOW, 80, 120),
                                    new PositionCommand(wristSubsystem, WristConstants.ALGAE_LOW)
                            ),
                            new PositionCommand(armSubsystem, ArmConstants.ALGAE_LOW)
                    ),
                    new SequentialCommandGroup( //Going up
                            new PositionCommand(armSubsystem, ArmConstants.ALGAE_LOW),
                            new ParallelCommandGroup(
                                    new PositionCommand(elevatorSubsystem, ElevatorConstants.ALGAE_LOW),
                                    new PositionCommand(wristSubsystem, WristConstants.ALGAE_LOW)
                            )
                    ),
                    () -> elevatorSubsystem.getMechM() > ElevatorConstants.ALGAE_LOW
            );
        }

        public static Command scoreBarge() {
            return new ConditionalCommand(
                    new SequentialCommandGroup( //Going down
                            new ParallelCommandGroup(
                                    new PositionCommand(elevatorSubsystem, ElevatorConstants.BARGE, 80, 120),
                                    new PositionCommand(wristSubsystem, WristConstants.BARGE)
                            ),
                            new PositionCommand(armSubsystem, ArmConstants.BARGE)
                    ),
                    new SequentialCommandGroup( //Going up
                            new PositionCommand(armSubsystem, ArmConstants.BARGE),
                            new ParallelCommandGroup(
                                    new PositionCommand(elevatorSubsystem, ElevatorConstants.BARGE),
                                    new PositionCommand(wristSubsystem, WristConstants.BARGE)
                            )
                    ),
                    () -> elevatorSubsystem.getMechM() > ElevatorConstants.BARGE
            );
        }

        public static Command scoreProcessor() {
            return new ConditionalCommand(
                    new SequentialCommandGroup( //Going down
                            new ParallelCommandGroup(
                                    new PositionCommand(elevatorSubsystem, ElevatorConstants.PROCESSOR, 80, 120),
                                    new PositionCommand(wristSubsystem, WristConstants.PROCESSOR)
                            ),
                            new PositionCommand(armSubsystem, ArmConstants.PROCESSOR)
                    ),
                    new SequentialCommandGroup( //Going up
                            new PositionCommand(armSubsystem, ArmConstants.PROCESSOR),
                            new ParallelCommandGroup(
                                    new PositionCommand(elevatorSubsystem, ElevatorConstants.PROCESSOR),
                                    new PositionCommand(wristSubsystem, WristConstants.PROCESSOR)
                            )
                    ),
                    () -> elevatorSubsystem.getMechM() > ElevatorConstants.PROCESSOR
            );
        }

        public static Command scoreL1() {
            return new SequentialCommandGroup(
                    new PositionCommand(armSubsystem, ArmConstants.L1),
                    new ParallelCommandGroup(
                            new PositionCommand(elevatorSubsystem, ElevatorConstants.L1),
                            new PositionCommand(wristSubsystem, WristConstants.L1)
                    )
            ).alongWith(new InstantCommand(() -> RobotContainer.setState(State.L1)));
        }


        public static Command scoreL2() {
            return new ConditionalCommand(
                    new SequentialCommandGroup( //Going down
                            new ParallelCommandGroup(
                                    new PositionCommand(elevatorSubsystem, ElevatorConstants.L2, 80, 120),
                                    new PositionCommand(wristSubsystem, WristConstants.L2)
                            ),
                            new PositionCommand(armSubsystem, ArmConstants.L2)
                    ),
                    new SequentialCommandGroup( //Going up
                            new PositionCommand(armSubsystem, ArmConstants.L2),
                            new ParallelCommandGroup(
                                    new PositionCommand(elevatorSubsystem, ElevatorConstants.L2),
                                    new PositionCommand(wristSubsystem, WristConstants.L2)
                                    // new VelocityCommand(intakeSubsystem, 50, 50)
                            )

                    ),
                    () -> elevatorSubsystem.getMechM() > ElevatorConstants.L2
            ).alongWith(new InstantCommand(() -> RobotContainer.setState(State.L2)));
        }

        public static Command scoreL3() {
            return new ConditionalCommand(
                    new SequentialCommandGroup( //Going down
                            new ParallelCommandGroup(
                                    new PositionCommand(elevatorSubsystem, ElevatorConstants.L3, 80, 120),
                                    new PositionCommand(wristSubsystem, WristConstants.L3)
                            ),
                            new PositionCommand(armSubsystem, ArmConstants.L3)
                    ),
                    new SequentialCommandGroup( //Going up
                            new PositionCommand(armSubsystem, ArmConstants.L3),
                            new ParallelCommandGroup(
                                    new PositionCommand(elevatorSubsystem, ElevatorConstants.L3),
                                    new PositionCommand(wristSubsystem, WristConstants.L3)
                            )
                    ),
                    () -> elevatorSubsystem.getMechM() > ElevatorConstants.L3
            ).alongWith(new InstantCommand(() -> RobotContainer.setState(State.L3)));
        }

        public static Command scoreL4() {
            return new SequentialCommandGroup(
                    new PositionCommand(armSubsystem, ArmConstants.L4),
                    new ParallelCommandGroup(
                            new PositionCommand(elevatorSubsystem, ElevatorConstants.L4),
                            new PositionCommand(wristSubsystem, WristConstants.L4)
                                    .beforeStarting(new WaitCommand(Double.MAX_VALUE)
                                            .until(() -> elevatorSubsystem.isMechGreaterThanPosition(.25)))
                    ).until(() -> elevatorSubsystem.isMechAtGoal(false)
                            && wristSubsystem.isMechAtGoal(false))
            ).alongWith(new InstantCommand(() -> RobotContainer.setState(State.L4)));
        }

        public static Command prepL4() {
            return new SequentialCommandGroup(
                    new PositionCommand(armSubsystem, ArmConstants.L4)
                            .alongWith(new PositionCommand(wristSubsystem, WristConstants.L3)),
                    new PositionCommand(elevatorSubsystem, .25)
            );
        }

        public static Command score() {
            return new SelectCommand<>(
                    Map.ofEntries(
                            Map.entry(State.L1, scoreL1()),
                            Map.entry(State.L2, scoreL2()),
                            Map.entry(State.L3, scoreL3()),
                            Map.entry(State.L4, scoreL4()),
                            Map.entry(State.ALGAE_LOW, removeAlgaeLow()),
                            Map.entry(State.ALGAE_HIGH, removeAlgaeHigh()),
                            Map.entry(State.BARGE, scoreBarge()),
                            Map.entry(State.PROCESSOR, scoreProcessor())
                    ),
                    RobotContainer::getState
            );
        }

        public static Command place() {
            return new ConditionalCommand(
                    new VelocityCommand(intakeSubsystem, -60, -60),
                    new ConditionalCommand(
                            new VelocityCommand(intakeSubsystem, 15, 5),
                            new ConditionalCommand(
                                    new VelocityCommand(intakeSubsystem, 75, 75),
                                    new ConditionalCommand(
                                            new VelocityCommand(intakeSubsystem, -100, -100),
                                            new VelocityCommand(intakeSubsystem, 20, 20),
                                            () -> RobotContainer.getState() == State.BARGE ||
                                                    RobotContainer.getState() == State.PROCESSOR
                                    ),
                                    () -> RobotContainer.getState() == State.L4
                            ),
                            () -> RobotContainer.getState() == State.L1
                    ),
                    () -> (RobotContainer.getState() == State.ALGAE_HIGH
                            || RobotContainer.getState() == State.ALGAE_LOW)
            );
        }

    }
}
