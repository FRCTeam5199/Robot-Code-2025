package frc.robot.commands;

import java.util.Map;

import com.ctre.phoenix6.swerve.SwerveRequest;

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
                        /*if ((!elevatorSubsystem.isMechAtGoal(false)
                                || !armSubsystem.isMechAtGoal(false)
                                || !wristSubsystem.isMechAtGoal(false))
                                && Math.abs(aprilTagSubsystem.getClosestTagXYYaw()[0]) < .3)
                            commandSwerveDrivetrain.setControl(
                                    drive.withVelocityX(0)
                                            .withVelocityY(yVelocity)
                                            .withRotationalRate(rotationVelocity));
                        else*/
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
                            RobotContainer::aligned,
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

//                        RobotContainer.commandSwerveDrivetrain.resetRotation(new Rotation2d(
//                                Math.toRadians(aprilTagSubsystem.getRotationToAlign(aprilTagSubsystem
//                                        .getClosestTagID()))));
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
                    RobotContainer.commandSwerveDrivetrain);
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

                        // RobotContainer.commandSwerveDrivetrain.resetRotation(new Rotation2d(
                        //     Math.toRadians(aprilTagSubsystem.getRotationToAlign(aprilTagSubsystem
                        //             .getClosestTagID()))));
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
                    RobotContainer.commandSwerveDrivetrain);
        }

        public static Command autoMoveForwardBottom() {
            return new FunctionalCommand(
                    () -> {
                    },
                    () -> {
                        if (DriverStation.getAlliance().isPresent()
                                && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue))
                            RobotContainer.commandSwerveDrivetrain.setControl(
                                    drive.withVelocityX(-.5)
                                            .withVelocityY(-.5));
                        else RobotContainer.commandSwerveDrivetrain.setControl(
                                drive.withVelocityX(-.5)
                                        .withVelocityY(.5));
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
                                    drive.withVelocityX(-0.5)
                                            .withVelocityY(0.5));
                        else RobotContainer.commandSwerveDrivetrain.setControl(
                                drive.withVelocityX(-0.5)
                                        .withVelocityY(-0.5));
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
                    new ParallelCommandGroup(
                            new PositionCommand(elevatorSubsystem, ElevatorConstants.STABLE, false)
                                    .until(() -> elevatorSubsystem.isAtBottom()
                                            && elevatorSubsystem.getMechM() < .05),
                            new PositionCommand(wristSubsystem, WristConstants.STABLE)
                                    .onlyIf(() -> (RobotContainer.getState() != State.ALGAE_HIGH
                                            && RobotContainer.getState() != State.ALGAE_LOW
                                            && RobotContainer.getState() != State.BARGE))
                    ),
                    new InstantCommand(() -> elevatorSubsystem.getMotor().setPosition(0)),
                    Arm.armStable()
            );
        }

        public static Command groundIntakeStable() {
            return new SequentialCommandGroup(
                    new ParallelCommandGroup(
                            new PositionCommand(elevatorSubsystem, 0, false)
                                    .until(() -> elevatorSubsystem.isAtBottom()
                                            && elevatorSubsystem.getMechM() < .05),
                            new PositionCommand(wristSubsystem, WristConstants.STABLE)
                    ),
                    new InstantCommand(() -> elevatorSubsystem.getMotor().setPosition(0)),
                    Arm.armStable()
            );
        }

        public static Command stableL4() {
            return new SequentialCommandGroup(
                    new PositionCommand(wristSubsystem, WristConstants.PREVIOUS_L4),
                    new ParallelCommandGroup(
                            new PositionCommand(elevatorSubsystem, ElevatorConstants.STABLE, false)
                                    .until(() -> elevatorSubsystem.isAtBottom()
                                            && elevatorSubsystem.getMechM() < .05),
                            new PositionCommand(wristSubsystem, WristConstants.STABLE)
                    ),
                    new InstantCommand(() -> elevatorSubsystem.getMotor().setPosition(0)),
                    Arm.armStable()
            );
        }

        public static Command stable() {
            return new ConditionalCommand(
                    stableL4(),
                    regularStable(),
                    () -> RobotContainer.getState() == State.L4
            );
        }

        public static Command intakeStable() {
            return new SequentialCommandGroup(
                    new PositionCommand(elevatorSubsystem, ElevatorConstants.STABLE, false)
                            .andThen(new PositionCommand(armSubsystem, ArmConstants.HP)),
                    new PositionCommand(wristSubsystem, WristConstants.STABLE)
            );
        }

        public static Command stableWithArm() {
            return new SequentialCommandGroup( //Won't clip elevator
                    new PositionCommand(elevatorSubsystem, ElevatorConstants.STABLE, false)
                            .until(() -> elevatorSubsystem.isAtBottom() && elevatorSubsystem.getMechM() < .1),
                    new InstantCommand(() -> elevatorSubsystem.getMotor().setPosition(0)),
                    new ParallelCommandGroup(
                            new PositionCommand(wristSubsystem, WristConstants.STABLE),
                            new PositionCommand(armSubsystem, ArmConstants.STABLE)
                    )
            );
        }

        public static Command wristandElevatorStable() {
            return new PositionCommand(elevatorSubsystem, ElevatorConstants.STABLE, false)
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
                    new PositionCommand(wristSubsystem, 0),
                    new ParallelCommandGroup(
                            zeroElevator(),
                            zeroArm()
                    ).withTimeout(5),
                    zeroWrist().withTimeout(2),
                    new WaitCommand(.5),
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
                            new PositionCommand(elevatorSubsystem, ElevatorConstants.GROUND, true),
                            new PositionCommand(wristSubsystem, WristConstants.GROUND)
                    ),
                    new SequentialCommandGroup(
                            new PositionCommand(elevatorSubsystem, ElevatorConstants.GROUND, false),
                            new ParallelCommandGroup(
                                    new PositionCommand(armSubsystem, ArmConstants.GROUND),
                                    new PositionCommand(wristSubsystem, WristConstants.GROUND)
                            )
                    ),
                    () -> elevatorSubsystem.getMechM() < ElevatorConstants.GROUND
            ).alongWith(groundIntakeSequence());
        }

        public static Command wristHP() {
            return new PositionCommand(wristSubsystem, WristConstants.HP);
        }

        public static Command elevatorHP() {
            return new PositionCommand(elevatorSubsystem, ElevatorConstants.HP);
        }

        public static Command intakeHP() {
            return new ConditionalCommand(
                    new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new PositionCommand(elevatorSubsystem, ElevatorConstants.HP, false)
                                            .until(() -> elevatorSubsystem.getMechM() < .4)
                                            .andThen(new PositionCommand(armSubsystem, ArmConstants.HP)),
                                    new PositionCommand(wristSubsystem, WristConstants.HP)
                            )
                    ),
                    new SequentialCommandGroup(
                            new ParallelCommandGroup( //Going up
                                    new PositionCommand(elevatorSubsystem, ElevatorConstants.HP, true),
                                    new PositionCommand(armSubsystem, ArmConstants.HP),
                                    new PositionCommand(wristSubsystem, WristConstants.HP)
                            )
                    ),
                    () -> elevatorSubsystem.getMechM() > ElevatorConstants.HP
            ).alongWith(new VelocityCommand(intakeSubsystem, 60)
                    /*.until(intakeSubsystem::hasCoralCurrent)
                    .andThen(new WaitCommand(1))
                    .andThen(new VelocityCommand(intakeSubsystem, 10)*/
                    .until(intakeSubsystem::hasCoral));
        }

        public static Command intakeSequence() {
            return new SequentialCommandGroup(
                    //Intake until beam initially breaks
                    new VelocityCommand(intakeSubsystem, 60)
                            .until(intakeSubsystem::hasCoral),
                    //Outtake slowly until beam connects
                    new VelocityCommand(intakeSubsystem, -15)
                            .until(() -> !intakeSubsystem.hasCoral()),
                    //Intake slowly until beam breaks; the coral is now barely at the beam
                    new VelocityCommand(intakeSubsystem, 15)
                            .until(intakeSubsystem::hasCoral),
                    //Intake slowly for an amount of time; lines it up completely
                    new VelocityCommand(intakeSubsystem, 10)
                            .withDeadline(new WaitCommand(0.2)),
                    //Outtake slowly until beam connects
                    new VelocityCommand(intakeSubsystem, -15)
                            .until(() -> !intakeSubsystem.hasCoral()),
                    //Intake slowly until beam breaks
                    new VelocityCommand(intakeSubsystem, 15)
                            .until(intakeSubsystem::hasCoral)
            );
        }

        public static Command reIntakeSequence() {
            //In case the coral gets stuck
            return new SequentialCommandGroup(
                    //Outtake slowly until beam connects
                    new VelocityCommand(intakeSubsystem, -50)
                            .until(() -> !intakeSubsystem.hasCoral()),
                    //Intake slowly until beam breaks; the coral is now barely at the beam
                    new VelocityCommand(intakeSubsystem, 25)
                            .until(intakeSubsystem::hasCoral),
                    //Intake slowly for an amount of time; lines it up completely
                    new VelocityCommand(intakeSubsystem, 10)
                            .withDeadline(new WaitCommand(0.2)),
                    //Outtake slowly until beam connects
                    new VelocityCommand(intakeSubsystem, -35)
                            .until(() -> !intakeSubsystem.hasCoral()),
                    //Intake slowly until beam breaks
                    new VelocityCommand(intakeSubsystem, 20)
                            .until(intakeSubsystem::hasCoral)
            );
        }

        public static Command groundIntakeSequence() {
            return new SequentialCommandGroup(
                    //Intake until beam initially breaks
                    new VelocityCommand(intakeSubsystem, 100)
                            .until(intakeSubsystem::hasCoral),
                    //Outtake slowly until beam connects
                    new VelocityCommand(intakeSubsystem, -25)
                            .until(() -> !intakeSubsystem.hasCoral()),
                    //Intake slowly until beam breaks; the coral is now barely at the beam
                    new VelocityCommand(intakeSubsystem, 15)
                            .until(intakeSubsystem::hasCoral)
            );
        }

        public static Command intakeAuto() {
            return new VelocityCommand(intakeSubsystem, 50).until(intakeSubsystem::hasCoral);
        }
    }


    public static class Arm {
        public static Command armGroundIntake() {
            return new PositionCommand(armSubsystem, ArmConstants.GROUND);
        }

        public static Command armL1() {
            return new PositionCommand(armSubsystem, ArmConstants.L1)
                    .alongWith(new InstantCommand(() -> RobotContainer.setState(State.L1)))
                    .alongWith(new VelocityCommand(intakeSubsystem, 0));
        }

        public static Command armL2() {
            return new PositionCommand(armSubsystem, ArmConstants.L2)
                    .alongWith(new InstantCommand(() -> RobotContainer.setState(State.L2)))
                    .alongWith(new VelocityCommand(intakeSubsystem, 0));
        }

        public static Command armL3() {
            return new PositionCommand(armSubsystem, ArmConstants.L3)
                    .alongWith(new InstantCommand(() -> RobotContainer.setState(State.L3)))
                    .alongWith(new VelocityCommand(intakeSubsystem, 0));
        }

        public static Command armL4() {
            return new PositionCommand(armSubsystem, ArmConstants.L4)
                    .alongWith(new InstantCommand(() -> RobotContainer.setState(State.L4)))
                    .alongWith(new VelocityCommand(intakeSubsystem, 0));
        }

        public static Command armAlgaeLow() {
            return new PositionCommand(armSubsystem, ArmConstants.ALGAE_LOW)
                    .alongWith(new InstantCommand(() -> RobotContainer.setState(State.ALGAE_LOW)));
        }

        public static Command armAlgaeHigh() {
            return new PositionCommand(armSubsystem, ArmConstants.ALGAE_HIGH)
                    .alongWith(new InstantCommand(() -> RobotContainer.setState(State.ALGAE_HIGH)));
        }

        public static Command armBarge() {
            return new PositionCommand(armSubsystem, ArmConstants.BARGE)
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
                            Map.entry(State.BARGE, armBarge())
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
                                    new PositionCommand(elevatorSubsystem, ElevatorConstants.ALGAE_HIGH, false),
                                    new PositionCommand(wristSubsystem, WristConstants.ALGAE_HIGH)
                            ),
                            new PositionCommand(armSubsystem, ArmConstants.ALGAE_HIGH)
                    ),
                    new SequentialCommandGroup( //Going up
                            new PositionCommand(armSubsystem, ArmConstants.ALGAE_HIGH),
                            new ParallelCommandGroup(
                                    new PositionCommand(elevatorSubsystem, ElevatorConstants.ALGAE_HIGH, true),
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
                                    new PositionCommand(elevatorSubsystem, ElevatorConstants.ALGAE_LOW, false),
                                    new PositionCommand(wristSubsystem, WristConstants.ALGAE_LOW)
                            ),
                            new PositionCommand(armSubsystem, ArmConstants.ALGAE_LOW)
                    ),
                    new SequentialCommandGroup( //Going up
                            new PositionCommand(armSubsystem, ArmConstants.ALGAE_LOW),
                            new ParallelCommandGroup(
                                    new PositionCommand(elevatorSubsystem, ElevatorConstants.ALGAE_LOW, true),
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
                                    new PositionCommand(elevatorSubsystem, ElevatorConstants.BARGE, false),
                                    new PositionCommand(wristSubsystem, WristConstants.BARGE)
                            ),
                            new PositionCommand(armSubsystem, ArmConstants.BARGE)
                    ),
                    new SequentialCommandGroup( //Going up
                            new PositionCommand(armSubsystem, ArmConstants.BARGE),
                            new ParallelCommandGroup(
                                    new PositionCommand(elevatorSubsystem, ElevatorConstants.BARGE, true),
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
                                    new PositionCommand(elevatorSubsystem, ElevatorConstants.PROCESSOR, false),
                                    new PositionCommand(wristSubsystem, WristConstants.PROCESSOR)
                            ),
                            new PositionCommand(armSubsystem, ArmConstants.PROCESSOR)
                    ),
                    new SequentialCommandGroup( //Going up
                            new PositionCommand(armSubsystem, ArmConstants.PROCESSOR),
                            new ParallelCommandGroup(
                                    new PositionCommand(elevatorSubsystem, ElevatorConstants.PROCESSOR, true),
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
                            new PositionCommand(elevatorSubsystem, ElevatorConstants.L1, true),
                            new PositionCommand(wristSubsystem, WristConstants.L1)
                    )
            ).alongWith(new InstantCommand(() -> RobotContainer.setState(State.L1)));
        }


        public static Command scoreL2() {
            return new ConditionalCommand(
                    new SequentialCommandGroup( //Going down
                            new ParallelCommandGroup(
                                    new PositionCommand(elevatorSubsystem, ElevatorConstants.L2, false),
                                    new PositionCommand(wristSubsystem, WristConstants.L2)
                            ),
                            new PositionCommand(armSubsystem, ArmConstants.L2)
                    ),
                    new SequentialCommandGroup( //Going up
                            new PositionCommand(armSubsystem, ArmConstants.L2),
                            new ParallelCommandGroup(
                                    new PositionCommand(elevatorSubsystem, ElevatorConstants.L2, true),
                                    new PositionCommand(wristSubsystem, WristConstants.L2)
                                    // new VelocityCommand(intakeSubsystem, 50)
                            )

                    ),
                    () -> elevatorSubsystem.getMechM() > ElevatorConstants.L2
            ).alongWith(new InstantCommand(() -> RobotContainer.setState(State.L2)));
        }

        public static Command scoreL3() {
            return new ConditionalCommand(
                    new SequentialCommandGroup( //Going down
                            new ParallelCommandGroup(
                                    new PositionCommand(elevatorSubsystem, ElevatorConstants.L3, false),
                                    new PositionCommand(wristSubsystem, WristConstants.L3)
                            ),
                            new PositionCommand(armSubsystem, ArmConstants.L3)
                    ),
                    new SequentialCommandGroup( //Going up
                            new PositionCommand(armSubsystem, ArmConstants.L3),
                            new ParallelCommandGroup(
                                    new PositionCommand(elevatorSubsystem, ElevatorConstants.L3, true),
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
//                            new PositionCommand(wristSubsystem, 10),
                            new PositionCommand(elevatorSubsystem, ElevatorConstants.L4, true),
                            new PositionCommand(wristSubsystem, WristConstants.L4)
                                    .beforeStarting(new WaitCommand(Double.MAX_VALUE)
                                            .until(() -> elevatorSubsystem.isMechGreaterThanPosition(.5)))
                    ).until(() -> elevatorSubsystem.isMechAtGoal(false)
                            && wristSubsystem.isMechAtGoal(false))
            ).alongWith(new InstantCommand(() -> RobotContainer.setState(State.L4)));
        }

        public static Command autoScoreL4() {
            return new SequentialCommandGroup(
                    new PositionCommand(armSubsystem, ArmConstants.L4),
                    new ParallelCommandGroup(
                            new PositionCommand(elevatorSubsystem, ElevatorConstants.L4, true),
                            new PositionCommand(wristSubsystem, WristConstants.L4)
                                    .beforeStarting(new WaitCommand(Double.MAX_VALUE)
                                            .until(() -> elevatorSubsystem.isMechGreaterThanPosition(.5)))
                    ).until(() -> elevatorSubsystem.isMechAtGoal(false)
                            && wristSubsystem.isMechAtGoal(false))
            ).alongWith(new InstantCommand(() -> RobotContainer.setState(State.L4)));
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
                            Map.entry(State.BARGE, scoreBarge())
                    ),
                    RobotContainer::getState
            );
        }

        public static Command place() {
            return new ConditionalCommand(
                    //L1
                    new VelocityCommand(intakeSubsystem, 25),
                    new ConditionalCommand(
                            //Barge
                            new SequentialCommandGroup(
                                    new InstantCommand(() -> intakeSubsystem.setScoringAlgae(true)),
                                    new InstantCommand(() -> intakeSubsystem.setVelocity(0)),
                                    new PositionCommand(wristSubsystem, 100)
                                            .until(() -> wristSubsystem.getDegrees() < 175),
                                    new VelocityCommand(intakeSubsystem, 50)
                            ),
                            new ConditionalCommand(
                                    //Processor
                                    new SequentialCommandGroup(
                                            new InstantCommand(() -> intakeSubsystem.setScoringAlgae(true)),
                                            new InstantCommand(() -> intakeSubsystem.setVelocity(50))
                                    ),
                                    //Everything else
                                    new VelocityCommand(intakeSubsystem, 50),
                                    () -> RobotContainer.getState() == State.PROCESSOR
                            ),
                            () -> RobotContainer.getState() == State.BARGE
                    ),
                    () -> RobotContainer.getState() == State.L1
            );
        }

    }
}
