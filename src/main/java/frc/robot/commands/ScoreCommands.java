package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.template.PositionCommand;
import frc.robot.subsystems.template.VelocityCommand;
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
    // public static double MaxAngularRate = TunerConstants.kRotationAt12Volts;

    private final static SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDesaturateWheelSpeeds(true) // Add a 10% deadband
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);

    public static Command stable() {
        return new ConditionalCommand(
                new SequentialCommandGroup( //Won't clip elevator
                        new ParallelCommandGroup(
                                new PositionCommand(wristSubsystem, 0),
                                new PositionCommand(elevatorSubsystem, 0, false)
                                        .until(() -> elevatorSubsystem.isAtBottom() && elevatorSubsystem.getMechM() < .1)
                        ),
                        new InstantCommand(() -> elevatorSubsystem.getMotor().setPosition(0)),
                        new PositionCommand(armSubsystem, 0.5)
                ),
                new SequentialCommandGroup( //Will clip elevator
                        new PositionCommand(wristSubsystem, 0),
                        new PositionCommand(elevatorSubsystem, 0, false)
                                .until(() -> elevatorSubsystem.isAtBottom() && elevatorSubsystem.getMechM() < .1),
                        new InstantCommand(() -> elevatorSubsystem.getMotor().setPosition(0)),
                        new PositionCommand(armSubsystem, 0.5)
                ),
                () -> wristSubsystem.getDegrees() < 50
        );
    }

    public static Command intakeHP() {
        return new ConditionalCommand(
                new SequentialCommandGroup( //Going down
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, 0.038, false),
                                new PositionCommand(wristSubsystem, 0.004)
                        ),
                        new PositionCommand(armSubsystem, 56.56)
                ),
                new SequentialCommandGroup( //Going up
                        new PositionCommand(armSubsystem, 56.56), //76,    .1
                        new PositionCommand(elevatorSubsystem, 0.038, true),
                        new PositionCommand(wristSubsystem, 0.004)
                ),
                () -> elevatorSubsystem.getMechM() > .15
        );
        // ).alongWith(new VelocityCommand(intakeSubsystem, 50));
    }

    public static Command intake() {
        return new FunctionalCommand(
                () -> {
                    timer.reset();
                },
                () -> intakeSubsystem.intake(),
                (interrupted) -> intakeSubsystem.stopIntake(),
                () -> {
                    if (intakeSubsystem.getStatorCurrent() > 35) {
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
                        new PositionCommand(elevatorSubsystem, 0, true),
                        new PositionCommand(wristSubsystem, 50)
                        // new VelocityCommand(intakeSubsystem, 50)
                )
        ).alongWith(new InstantCommand(() -> RobotContainer.setState(State.L1)));
    }

    public static Command scoreL2() {
        return new ConditionalCommand(
                new SequentialCommandGroup( //Going down
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .04, false),
                                new PositionCommand(wristSubsystem, 152)
                                // new VelocityCommand(intakeSubsystem, 50)
                        ),
                        new PositionCommand(armSubsystem, 74)
                ),
                new SequentialCommandGroup( //Going up
                        new PositionCommand(armSubsystem, 74),
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .04, true),
                                new PositionCommand(wristSubsystem, 152)
                                // new VelocityCommand(intakeSubsystem, 50)
                        )
                ),
                () -> elevatorSubsystem.getMechM() > .07
        ).alongWith(new InstantCommand(() -> RobotContainer.setState(State.L2)));
    }

    public static Command scoreL3() {
        return new ConditionalCommand(
                new SequentialCommandGroup( //Going down
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .36, false),
                                new PositionCommand(wristSubsystem, 155)
                                // new VelocityCommand(intakeSubsystem, 50)
                        ),
                        new PositionCommand(armSubsystem, 85)
                ),
                new SequentialCommandGroup( //Going up
                        new PositionCommand(armSubsystem, 85),
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .36, true),
                                new PositionCommand(wristSubsystem, 155)
                                // new VelocityCommand(intakeSubsystem, 50)
                        )
                ),
                () -> elevatorSubsystem.getMechM() > .33
        ).alongWith(new InstantCommand(() -> RobotContainer.setState(State.L3)));
    }

    public static Command scoreL4() {
        return new SequentialCommandGroup(
                new PositionCommand(armSubsystem, 87),
                new ParallelCommandGroup(
                        new PositionCommand(elevatorSubsystem, 1.003, true),
                        new PositionCommand(wristSubsystem, 152)
                )
        ).alongWith(new InstantCommand(() -> RobotContainer.setState(State.L4)));
    }

    public static Command score() {
        return new SelectCommand<>(
                Map.ofEntries(
                        Map.entry(State.L1, scoreL1()),
                        Map.entry(State.L2, scoreL2()),
                        Map.entry(State.L3, scoreL3()),
                        Map.entry(State.L4, scoreL4())
                ),
                RobotContainer::getState
        );
    }

    public static Command scoreL2NoDunk() {
        return new ConditionalCommand(
                new SequentialCommandGroup( //Going down
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .08, false),
                                new PositionCommand(wristSubsystem, 152)
                                // new VelocityCommand(intakeSubsystem, 50)
                        ),
                        new PositionCommand(armSubsystem, 71)
                ),
                new SequentialCommandGroup( //Going up
                        new PositionCommand(armSubsystem, 71),
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .08, true),
                                new PositionCommand(wristSubsystem, 152)
                                // new VelocityCommand(intakeSubsystem, 50)
                        )

                ),
                () -> elevatorSubsystem.getMechM() > .07
        ).alongWith(new InstantCommand(() -> RobotContainer.setState(State.L2)));
    }

    public static Command scoreL3NoDunk() {
        return new ConditionalCommand(
                new SequentialCommandGroup( //Going down
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .396, false),
                                new PositionCommand(wristSubsystem, 155)
                                // new VelocityCommand(intakeSubsystem, 50)
                        ),
                        new PositionCommand(armSubsystem, 82)
                ),
                new SequentialCommandGroup( //Going up
                        new PositionCommand(armSubsystem, 82),
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .396, true),
                                new PositionCommand(wristSubsystem, 155)
                                // new VelocityCommand(intakeSubsystem, 50)
                        )
                ),
                () -> elevatorSubsystem.getMechM() > .33
        ).alongWith(new InstantCommand(() -> RobotContainer.setState(State.L3)));
    }

    public static Command scoreL4NoDunk() {
        return new SequentialCommandGroup(
                new PositionCommand(armSubsystem, 85),
                new ParallelCommandGroup(
                        new PositionCommand(elevatorSubsystem, 1.003, true),
                        new PositionCommand(wristSubsystem, 161)
                        // new VelocityCommand(intakeSubsystem, 50)
                )
        ).alongWith(new InstantCommand(() -> RobotContainer.setState(State.L4)));
    }

    public static Command scoreNoDunk() {
        return new SelectCommand<>(
                Map.ofEntries(
                        Map.entry(State.L1, scoreL1()),
                        Map.entry(State.L2, scoreL2NoDunk()),
                        Map.entry(State.L3, scoreL3NoDunk()),
                        Map.entry(State.L4, scoreL4NoDunk())
                ),
                RobotContainer::getState
        );
    }

    public static Command armL2() {
        return new PositionCommand(armSubsystem, 74).alongWith(
                new InstantCommand(() -> RobotContainer.setState(State.L2)));
    }

    public static Command armL3() {
        return new PositionCommand(armSubsystem, 85)
                .alongWith(new InstantCommand(() -> RobotContainer.setState(State.L3)));
    }

    public static Command armL4() {
        return new PositionCommand(armSubsystem, 84)
                .alongWith(new InstantCommand(() -> RobotContainer.setState(State.L4)));
    }

    public static Command dunk() {
        return new SelectCommand<>(
                Map.ofEntries(
                        Map.entry(State.L2, new PositionCommand(armSubsystem, 65)),
                        Map.entry(State.L3, new PositionCommand(armSubsystem, 73)),
                        Map.entry(State.L4, new PositionCommand(wristSubsystem, 168))
                ),
                RobotContainer::getState
        );
    }

    public static Command zeroElevator() {
        return new FunctionalCommand(
                () -> elevatorSubsystem.setVoltage(-2),
                () -> {
                },
                (interrupted) -> {
                    elevatorSubsystem.setVoltage(0);
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
                },
                armSubsystem::isAtBottom,
                armSubsystem
        );
    }

    public static Command zeroSubsystems() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        zeroElevator(),
                        zeroArm(),
                        zeroWrist()
                ),
                new WaitCommand(.75),
                new ParallelCommandGroup(
                        new InstantCommand(() -> elevatorSubsystem.getMotor().setPosition(0)),
                        new InstantCommand(() -> armSubsystem.getMotor().setPosition(0)),
                        new InstantCommand(() -> wristSubsystem.getMotor().setPosition(0))
                ),
                new ParallelCommandGroup(
                        new PositionCommand(elevatorSubsystem, 0, true),
                        new PositionCommand(armSubsystem, 0),
                        new PositionCommand(wristSubsystem, 0)
                )
        );
    }

    public static Command autoAlignLAuton() {
//            Rotation2d og = RobotContainer.commandSwerveDrivetrain.getPose().getRotation().plus(new Rotation2d(Math.PI));
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
                            drive.withVelocityX(-0.2)
                                    .withVelocityY(-0.2));

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


//    public static Command algaeL1() {
//        return new ConditionalCommand(
//                new SequentialCommandGroup(
//                        new ParallelCommandGroup(
////                                new PositionCommand(elevatorSubsystem, .2, 20, 50),
//                                new PositionCommand(wristSubsystem, 132)
//                        ),
//                        new PositionCommand(armSubsystem, 64.71)
//                ),
//                new SequentialCommandGroup(
//                        new PositionCommand(armSubsystem, 64.71),
//                        new ParallelCommandGroup(
////                                new PositionCommand(elevatorSubsystem, .2, 20, 50),
//                                new PositionCommand(wristSubsystem, 132)
//                        )
//
//                ),
//                () -> elevatorSubsystem.getMechM() > .06
//        );
//    }
//
//    public static Command algaeL2() {
//        return new ConditionalCommand(
//                new SequentialCommandGroup( //Going down
//                        new ParallelCommandGroup(
////                                new PositionCommand(elevatorSubsystem, 0.5, 36, 20),
//                                new PositionCommand(wristSubsystem, 140)
//                        ),
//                        new PositionCommand(armSubsystem, 72.21)
//                ),
//                new SequentialCommandGroup( //Going up
//                        new PositionCommand(armSubsystem, 72.21),
//                        new ParallelCommandGroup(
////                                new PositionCommand(elevatorSubsystem, 0.5, 60, 20),
//                                new PositionCommand(wristSubsystem, 140)
//                        )
//                ),
//                () -> elevatorSubsystem.getMechM() > .4
//        );
//    }
//
//    public static Command algaeStable() {
//        return new ConditionalCommand(
//                new SequentialCommandGroup( //Won't clip elevator
//                        new ParallelCommandGroup(
//                                new PositionCommand(wristSubsystem, 56)
////                                new PositionCommand(elevatorSubsystem, 0, 40, 20)
//                        ),
//                        new PositionCommand(armSubsystem, 0.5)
//                ),
//                new SequentialCommandGroup( //Will clip elevator
//                        new PositionCommand(wristSubsystem, 56),
////                        new PositionCommand(elevatorSubsystem, 0, 40, 20),
//                        new PositionCommand(armSubsystem, 0.5)
//                ),
//                () -> wristSubsystem.getDegrees() < 50
//        );
//    }

}
