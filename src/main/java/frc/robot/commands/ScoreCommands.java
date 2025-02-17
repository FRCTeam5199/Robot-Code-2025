package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.template.PositionCommand;
import frc.robot.utility.State;

import java.util.Map;

import static frc.robot.RobotContainer.turnPIDController;
import static frc.robot.RobotContainer.yVelocity;

public class ScoreCommands {
    private static ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private static ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();
    private static WristSubsystem wristSubsystem = WristSubsystem.getInstance();
    private static IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();

    private static AprilTagSubsystem aprilTagSubsystem = AprilTagSubsystem.getInstance();

    private static Timer timer = new Timer();

    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.baseUnitMagnitude();
    public static double MaxAngularRate = TunerConstants.kRotationAt12Volts;

    private final static SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDesaturateWheelSpeeds(true) // Add a 10% deadband
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);

    public static Command intakeHP() {
        return new ConditionalCommand(
                new SequentialCommandGroup( //Going down
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, 0.24, 30, 150),
                                new PositionCommand(wristSubsystem, 0.7)
                        ),
                        new PositionCommand(armSubsystem, 74.48)
                ),
                new SequentialCommandGroup( //Going up
                        new PositionCommand(armSubsystem, 74.48),
                        new PositionCommand(elevatorSubsystem, 0.24, 105, 180),
                        new PositionCommand(wristSubsystem, 0.63)
                ),
                () -> elevatorSubsystem.getMechM() > .24
        ).alongWith(new InstantCommand(() -> intakeSubsystem.setPercent(0)));
    }

    public static Command intake() {
        return new FunctionalCommand(
                () -> {
                    timer.reset();
                },
                () -> intakeSubsystem.intake(),
                (interrupted) -> intakeSubsystem.stopIntake(),
                () -> {
                    if (intakeSubsystem.getStatorCurrent() > 25) {
                        timer.start();
                        if (timer.hasElapsed(.1)) {
                            timer.stop();
                            return true;
                        }
                        return false;
                    }
                    return false;
                },
                intakeSubsystem);
    }

    public static Command outtake() {
        return new FunctionalCommand(
                () -> {
                    timer.reset();
                },
                () -> intakeSubsystem.outtake(),
                (bool) -> intakeSubsystem.stopIntake(),
                () -> {
                    timer.start();
                    if (timer.hasElapsed(.3)) {
                        timer.stop();
                        return true;

                    }
                    return false;
                },
                intakeSubsystem);
    }

    public static Command scoreL1() {
        return new SequentialCommandGroup(
                new PositionCommand(armSubsystem, 0),
                new ParallelCommandGroup(
                        new PositionCommand(elevatorSubsystem, 0, 50, 100),
                        new PositionCommand(wristSubsystem, 25),
                        new InstantCommand(() -> intakeSubsystem.setPercent(.5))
                )
        ).alongWith(new InstantCommand(() -> RobotContainer.setState(State.L1)));
    }

    public static Command scoreL2() {
        return new ConditionalCommand(
                new SequentialCommandGroup( //Going down
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .05, 20, 50),
                                new PositionCommand(wristSubsystem, 71),
                                new InstantCommand(() -> intakeSubsystem.setPercent(.1))
                        ),
                        new PositionCommand(armSubsystem, 71)
                ),
                new SequentialCommandGroup( //Going up
                        new PositionCommand(armSubsystem, 71),
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .05, 20, 50),
                                new PositionCommand(wristSubsystem, 71),
                                new InstantCommand(() -> intakeSubsystem.setPercent(.5))
                        )

                ),
                () -> elevatorSubsystem.getMechM() > .07
        ).alongWith(new InstantCommand(() -> RobotContainer.setState(State.L2)));
    }

    public static Command scoreL3() {
        return new ConditionalCommand(
                new SequentialCommandGroup( //Going down
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .33, 36, 20),
                                new PositionCommand(wristSubsystem, 73),
                                new InstantCommand(() -> intakeSubsystem.setPercent(.8))
                        ),
                        new PositionCommand(armSubsystem, 84)
                ),
                new SequentialCommandGroup( //Going up
                        new PositionCommand(armSubsystem, 84),
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .33, 60, 20),
                                new PositionCommand(wristSubsystem, 73),
                                new InstantCommand(() -> intakeSubsystem.setPercent(.8))
                        )
                ),
                () -> elevatorSubsystem.getMechM() > .33
        ).alongWith(new InstantCommand(() -> RobotContainer.setState(State.L3)));
    }

    public static Command scoreL4() {
        return new SequentialCommandGroup(
                new PositionCommand(armSubsystem, 82),
                new ParallelCommandGroup(
                        new PositionCommand(elevatorSubsystem, .955, 90, 40),
                        new PositionCommand(wristSubsystem, 77),
                        new InstantCommand(() -> intakeSubsystem.setPercent(.5))
                )
        ).alongWith(new InstantCommand(() -> RobotContainer.setState(State.L4)));
    }

    public static Command armL2() {
        return new PositionCommand(armSubsystem, 71)
                .alongWith(new InstantCommand(() -> intakeSubsystem.setPercent(.5)));
    }

    public static Command armL3() {
        return new PositionCommand(armSubsystem, 84)
                .alongWith(new InstantCommand(() -> intakeSubsystem.setPercent(.5)));
    }

    public static Command armL4() {
        return new PositionCommand(armSubsystem, 84)
                .alongWith(new InstantCommand(() -> intakeSubsystem.setPercent(.5)));
    }


    public static Command stable() {
        return new ConditionalCommand(
                new SequentialCommandGroup( //Won't clip elevator
                        new ParallelCommandGroup(
                                new PositionCommand(wristSubsystem, 0),
                                new PositionCommand(elevatorSubsystem, 0, 40, 20)
                                        .until(() -> elevatorSubsystem.isAtBottom() && elevatorSubsystem.getMechM() < .1)
                        ),
                        new InstantCommand(() -> elevatorSubsystem.getMotor().setPosition(0)),
                        new PositionCommand(armSubsystem, 0.5)
                ),
                new SequentialCommandGroup( //Will clip elevator
                        new PositionCommand(wristSubsystem, 0),
                        new PositionCommand(elevatorSubsystem, 0, 40, 20)
                                .until(() -> elevatorSubsystem.isAtBottom() && elevatorSubsystem.getMechM() < .1),
                        new InstantCommand(() -> elevatorSubsystem.getMotor().setPosition(0)),
                        new PositionCommand(armSubsystem, 0.5)
                ),
                () -> wristSubsystem.getDegrees() < 50
        );
    }


    public static Command dunk() {
        return new SelectCommand<>(
                Map.ofEntries(
                        Map.entry(State.L2, new PositionCommand(armSubsystem, 65)),
                        Map.entry(State.L3, new PositionCommand(armSubsystem, 73))
                ),
                RobotContainer::getState
        );
    }

    public static Command algaeL1() {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .2, 20, 50),
                                new PositionCommand(wristSubsystem, 66.37)
                        ),
                        new PositionCommand(armSubsystem, 64.71)
                ),
                new SequentialCommandGroup(
                        new PositionCommand(armSubsystem, 64.71),
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .2, 20, 50),
                                new PositionCommand(wristSubsystem, 66.37)
                        )

                ),
                () -> elevatorSubsystem.getMechM() > .06
        );
    }

    public static Command algaeL2() {
        return new ConditionalCommand(
                new SequentialCommandGroup( //Going down
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, 0.5, 36, 20),
                                new PositionCommand(wristSubsystem, 65.8)
                        ),
                        new PositionCommand(armSubsystem, 72.21)
                ),
                new SequentialCommandGroup( //Going up
                        new PositionCommand(armSubsystem, 72.21),
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, 0.5, 60, 20),
                                new PositionCommand(wristSubsystem, 65.8)
                        )
                ),
                () -> elevatorSubsystem.getMechM() > .4
        );
    }

    public static Command algaeStable() {
        return new ConditionalCommand(
                new SequentialCommandGroup( //Won't clip elevator
                        new ParallelCommandGroup(
                                new PositionCommand(wristSubsystem, 28),
                                new PositionCommand(elevatorSubsystem, 0, 40, 20)
                        ),
                        new PositionCommand(armSubsystem, 0.5)
                ),
                new SequentialCommandGroup( //Will clip elevator
                        new PositionCommand(wristSubsystem, 28),
                        new PositionCommand(elevatorSubsystem, 0, 40, 20),
                        new PositionCommand(armSubsystem, 0.5)
                ),
                () -> wristSubsystem.getDegrees() < 50
        );
    }

    public static Command zeroElevator() {
        return new FunctionalCommand(
                () -> elevatorSubsystem.setVoltage(-2),
                () -> {
                },
                (interrupted) -> {
                    elevatorSubsystem.setVoltage(0);
                    elevatorSubsystem.getMotor().setPosition(0);
                },
                elevatorSubsystem::isAtBottom,
                elevatorSubsystem
        );
    }

    public static Command zeroWrist() {
        return new FunctionalCommand(
                () -> wristSubsystem.setVoltage(-1),
                () -> {
                },
                (interrupted) -> {
                    wristSubsystem.setVoltage(0);
                    wristSubsystem.getMotor().setPosition(0);
                },
                wristSubsystem::isAtBottom,
                wristSubsystem
        );
    }


    public static Command zeroArm() {
        return new FunctionalCommand(
                () -> armSubsystem.setVoltage(-2),
                () -> {
                },
                (interrupted) -> {
                    armSubsystem.setVelocity(0);
                    armSubsystem.getMotor().setPosition(-1);
                },
                armSubsystem::isAtBottom,
                armSubsystem
        );
    }

    public static Command zeroSubsystems() {
        return new ParallelCommandGroup(
                zeroElevator(),
                zeroArm(),
                zeroWrist()
        );
    }


    public static Command autoAlignLAuton() {
        //    Rotation2d og = RobotContainer.commandSwerveDrivetrain.getPose().getRotation().plus(new Rotation2d(Math.PI));

        return new FunctionalCommand(
                () -> {

                    if (RobotContainer.autoAlignYOffset < 0) {
                        RobotContainer.autoAlignYOffset = -RobotContainer.autoAlignYOffset;
                    }
                    System.out.println("Starting");

                    RobotContainer.commandSwerveDrivetrain.resetRotation(new Rotation2d(
                            Math.toRadians(aprilTagSubsystem.getRotationToAlign(aprilTagSubsystem
                                    .getClosestTagID()))));

                },
                () -> {


                    RobotContainer.commandSwerveDrivetrain.setControl(
                            drive.withVelocityX(RobotContainer.xVelocity)
                                    .withVelocityY(yVelocity)
                                    .withRotationalRate(turnPIDController.calculate(
                                            RobotContainer.commandSwerveDrivetrain.getPose().getRotation().getDegrees(), 0)));

                    System.out.println("Going");

                },
                (bool) -> {

                    //     RobotContainer.commandSwerveDrivetrain
                    //             .resetRotation(og);
                    RobotContainer.commandSwerveDrivetrain
                            .resetRotation(new Rotation2d(Math.toRadians(RobotContainer.commandSwerveDrivetrain
                                    .getPigeon2().getRotation2d().getDegrees() + 180)));


                    System.out.println("ending");


                }, RobotContainer::aligned,
                RobotContainer.commandSwerveDrivetrain);
    }

    public static Command autoAlignRAuton() {

        return new FunctionalCommand(
                () -> {
                    if (RobotContainer.autoAlignYOffset > 0) {
                        RobotContainer.autoAlignYOffset = -RobotContainer.autoAlignYOffset;
                    }
                    System.out.println("Starting");

                    RobotContainer.commandSwerveDrivetrain.resetRotation(new Rotation2d(
                            Math.toRadians(aprilTagSubsystem.getRotationToAlign(aprilTagSubsystem
                                    .getClosestTagID()))));

                },
                () -> {


                    RobotContainer.commandSwerveDrivetrain.setControl(
                            drive.withVelocityX(RobotContainer.xVelocity)
                                    .withVelocityY(yVelocity)
                                    .withRotationalRate(turnPIDController.calculate(
                                            RobotContainer.commandSwerveDrivetrain.getPose().getRotation().getDegrees(), 0)));

                    System.out.println("Going");

                },
                (bool) -> {

                    RobotContainer.commandSwerveDrivetrain
                            .resetRotation(new Rotation2d(Math.toRadians(RobotContainer.commandSwerveDrivetrain
                                    .getPigeon2().getRotation2d().getDegrees() + 180)));
                    // commandSwerveDrivetrain
                    // .resetRotation(new Rotation2d(Math.toRadians(commandSwerveDrivetrain
                    // .getPigeon2().getRotation2d().getDegrees())));


                    System.out.println("ending");


                }, RobotContainer::aligned,
                RobotContainer.commandSwerveDrivetrain);
    }

    public static Command autoMoveForward() {

        return new FunctionalCommand(
                () -> {
                    System.out.println("Forward Start");
                },
                () -> {
                    RobotContainer.commandSwerveDrivetrain.setControl(
                            drive.withVelocityX(-.4)
                                    .withVelocityY(-.4));

                    System.out.println("Going Forward");

                },
                (bool) -> {
                    RobotContainer.commandSwerveDrivetrain.setControl(
                            drive.withVelocityX(0)
                                    .withVelocityY(0));

                    System.out.println("ending");
                },
                () -> {
                    if (intakeSubsystem.getStatorCurrent() > 25) {
                        timer.start();
                        if (timer.hasElapsed(.1)) {
                            timer.stop();
                            return true;
                        }
                        return false;
                    }
                    return false;
                },
                RobotContainer.commandSwerveDrivetrain);
    }
}
