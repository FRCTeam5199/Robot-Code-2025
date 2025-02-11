package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.template.PositionCommand;
import frc.robot.subsystems.template.VelocityCommand;

public class ScoreCommands {
    private static ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private static ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();
    private static WristSubsystem wristSubsystem = WristSubsystem.getInstance();
    private static IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    private static Timer timer = new Timer();

    public static Command intakeHP() {
        return new ConditionalCommand(
                new SequentialCommandGroup( //Going down
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, 0.25, 30, 150),
                                new PositionCommand(wristSubsystem, 0.7)
                        ),
                        new PositionCommand(armSubsystem, 74.48)
                ),
                new SequentialCommandGroup( //Going up
                        new PositionCommand(armSubsystem, 74.48),
                        new PositionCommand(elevatorSubsystem, 0.25, 105, 180),
                        new PositionCommand(wristSubsystem, 0.63)
                ),
                () -> elevatorSubsystem.getMechM() > .25
        );
    }

    public static Command intake() {
        return new FunctionalCommand(
                () -> {
                    timer.reset();
                },
                () -> intakeSubsystem.intake(),
                (bool) -> intakeSubsystem.stopIntake(),
                () -> {
                    if (intakeSubsystem.getStatorCurrent() > 50) {
                        timer.start();
                        if (timer.hasElapsed(.5)) {
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
                    if (timer.hasElapsed(.2)) {
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
                        new PositionCommand(wristSubsystem, 25)
                )
        );
    }

    public static Command scoreL2() {
        return new ConditionalCommand(
                new SequentialCommandGroup( //Going down
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .1, 20, 50), //36, 180
                                new PositionCommand(wristSubsystem, 72)
                        ),
                        new PositionCommand(armSubsystem, 63)
                ),
                new SequentialCommandGroup( //Going up
                        new PositionCommand(armSubsystem, 63),
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .1, 20, 50), //105, 180
                                new PositionCommand(wristSubsystem, 72)
                        )

                ),
                () -> elevatorSubsystem.getMechM() > .06
        );
    }

    public static Command scoreL3() {
        return new ConditionalCommand(
                new SequentialCommandGroup( //Going down
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .4, 36, 20), //36, 180
                                new PositionCommand(wristSubsystem, 73)
                        ),
                        new PositionCommand(armSubsystem, 72.21)
                ),
                new SequentialCommandGroup( //Going up
                        new PositionCommand(armSubsystem, 72.21),
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .4, 60, 20), //75, 360
                                new PositionCommand(wristSubsystem, 73)
                        )
                ),
                () -> elevatorSubsystem.getMechM() > .4
        );
    }

    public static Command scoreL4() {
        return new SequentialCommandGroup(
                new PositionCommand(armSubsystem, 82),
                new ParallelCommandGroup(
                        new PositionCommand(elevatorSubsystem, 0.87, 60, 20),//120, 360
                        new PositionCommand(wristSubsystem, 73)
                )
        );
    }

    public static Command armL2() {
        return new SequentialCommandGroup(
                new PositionCommand(armSubsystem, 64.71)
        );
    }

    public static Command armL3() {
        return new SequentialCommandGroup(
                new PositionCommand(armSubsystem, 72.21)
        );
    }

    public static Command armL4() {
        return new SequentialCommandGroup(
                new PositionCommand(armSubsystem, 76)
        );
    }


    public static Command stable() {
        return new ConditionalCommand(
                new SequentialCommandGroup( //Won't clip elevator
                        new ParallelCommandGroup(
                                new PositionCommand(wristSubsystem, 0),
                                new PositionCommand(elevatorSubsystem, 0, 40, 20)
                        ),
                        new PositionCommand(armSubsystem, 0.5)
                ),
                new SequentialCommandGroup( //Will clip elevator
                        new PositionCommand(wristSubsystem, 0),
                        new PositionCommand(elevatorSubsystem, 0, 40, 20),
                        new PositionCommand(armSubsystem, 0.5)
                ),
                () -> wristSubsystem.getDegrees() < 50
        );
    }


    public static Command dunk() {
        return new SequentialCommandGroup(

        );
    }

    public static Command algaeL1() {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .2, 20, 50), //36, 180
                                new PositionCommand(wristSubsystem, 66.37)
                        ),
                        new PositionCommand(armSubsystem, 64.71)
                ),
                new SequentialCommandGroup(
                        new PositionCommand(armSubsystem, 64.71),
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .2, 20, 50), //105, 180
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
                                new PositionCommand(elevatorSubsystem, 0.5, 36, 20), //36, 180
                                new PositionCommand(wristSubsystem, 65.8) //30
                        ),
                        new PositionCommand(armSubsystem, 72.21)
                ),
                new SequentialCommandGroup( //Going up
                        new PositionCommand(armSubsystem, 72.21),
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, 0.5, 60, 20), //75, 360
                                new PositionCommand(wristSubsystem, 65.8) //30
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
                () -> elevatorSubsystem.setVelocity(-10),
                () -> {
                },
                interrupted -> {
                    elevatorSubsystem.setVelocity(0);
                    elevatorSubsystem.getMotor().setPosition(0);
                },
                elevatorSubsystem::isAtBottom,
                elevatorSubsystem
        );
    }


}
