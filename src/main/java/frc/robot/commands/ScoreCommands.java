package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.template.PositionCommand;

public class ScoreCommands {
    private static ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private static ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();
    private static WristSubsystem wristSubsystem = WristSubsystem.getInstance();
            
            
    public static Command intakeHP() {
        //Was -- elevator = .13, arm = 77, wrist = 85 [adding 5 to wrist]
        return new ConditionalCommand(
                new SequentialCommandGroup( //Going down
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, 0.156, 36, 180),
                                new PositionCommand(wristSubsystem, 0.5)
                        ),
                        new PositionCommand(armSubsystem, 74.48)
                ),
                new SequentialCommandGroup( //Going up
                        new PositionCommand(armSubsystem, 74.48),
                        new PositionCommand(elevatorSubsystem, .156, 105, 180),
                        new PositionCommand(wristSubsystem, 0.5)
                ),
                () -> elevatorSubsystem.getMechM() > .3
        );
    }

    public static Command scoreL1(){ 
        return new ConditionalCommand(
                new SequentialCommandGroup( //Going down
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, 0.013, 36, 180),
                                new PositionCommand(wristSubsystem, 59.66)
                        ),
                        new PositionCommand(armSubsystem, 44.12)
                ),
                new SequentialCommandGroup( //Going up
                        new PositionCommand(armSubsystem, 44.12),
                        new PositionCommand(elevatorSubsystem, .013, 105, 180),
                        new PositionCommand(wristSubsystem, 59.66)
                ),
                () -> elevatorSubsystem.getMechM() > .3
        );
    }

    public static Command scoreL2(){
        return new ConditionalCommand(
                new SequentialCommandGroup( //Going down
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .151, 20, 50), //36, 180
                                new PositionCommand(wristSubsystem, 66.37)
                                ),
                        new PositionCommand(armSubsystem, 64.71)
                ),
                new SequentialCommandGroup( //Going up
                        new PositionCommand(armSubsystem, 64.71),
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .151, 20, 50), //105, 180
                                new PositionCommand(wristSubsystem, 66.37)
                        )

                ),
                () -> elevatorSubsystem.getMechM() > .06
        );
    }

    public static Command scoreL3(){
        return new ConditionalCommand(
                new SequentialCommandGroup( //Going down
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem,  0.44, 36, 20), //36, 180
                                new PositionCommand(wristSubsystem, 65.8) //30
                         ),
                        new PositionCommand(armSubsystem, 72.21)
                ),
                new SequentialCommandGroup( //Going up
                        new PositionCommand(armSubsystem, 72.21),
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem,  0.44, 60, 20), //75, 360
                                new PositionCommand(wristSubsystem, 65.8) //30
                        )
                ),
                () -> elevatorSubsystem.getMechM() > .4
        );
    }

    //hotsunay makoe in fortnite battle royale
    public static Command scoreL4(){
        return new SequentialCommandGroup(
                new PositionCommand(armSubsystem, 80.874),
                new ParallelCommandGroup(
                        new PositionCommand(elevatorSubsystem, 0.929, 60, 20),//120, 360
                        new PositionCommand(wristSubsystem, 64)//27.76
                )
        );
    }

    public static Command stable(){
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


    public static Command dunk(){
        return new SequentialCommandGroup(
       
        );
    }
    
    public static Command algaeL1(){ 
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, 0.1, 36, 180),
                                new PositionCommand(wristSubsystem, 59.66)
                        ),
                        new PositionCommand(armSubsystem, 44.12)
                ),
                new SequentialCommandGroup(
                        new PositionCommand(armSubsystem, 44.12),
                        new PositionCommand(elevatorSubsystem, .1, 105, 180),
                        new PositionCommand(wristSubsystem, 59.66)
                ),
                () -> elevatorSubsystem.getMechM() > .3
        );
    }

    public static Command algaeL2(){
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
    public static Command algaeStable(){
        return new ConditionalCommand(
                new SequentialCommandGroup( //Won't clip elevator
                        new ParallelCommandGroup(
                                new PositionCommand(wristSubsystem, 15),
                                new PositionCommand(elevatorSubsystem, 0, 40, 20)
                        ),
                        new PositionCommand(armSubsystem, 0.5)
                ),
                new SequentialCommandGroup( //Will clip elevator
                        new PositionCommand(wristSubsystem, 15),
                        new PositionCommand(elevatorSubsystem, 0, 40, 20),
                        new PositionCommand(armSubsystem, 0.5)
                ),
                () -> wristSubsystem.getDegrees() < 50
        );
    }
}
