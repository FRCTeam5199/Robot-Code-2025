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

        //Was -- elevator = .13, arm = 77, wrist = 85

        return new ConditionalCommand(
                new SequentialCommandGroup( //Going down
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .314, 40, 200),
                                new PositionCommand(wristSubsystem, 95)
                        ),
                        new PositionCommand(armSubsystem, 77)
                ),
                new SequentialCommandGroup( //Going up
                        new PositionCommand(armSubsystem, 77),
                        new PositionCommand(elevatorSubsystem, .314, 115, 200),
                        new PositionCommand(wristSubsystem, 95)
                ),
                () -> elevatorSubsystem.getMechM() > .3
        );
    }

    public static Command scoreL1(){
        return new SequentialCommandGroup(
  
        );
    }

    public static Command scoreL2(){
        return new ConditionalCommand(
                new SequentialCommandGroup( //Going down
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .067, 40, 200),
                                new PositionCommand(wristSubsystem, 18)
                                ),
                        new PositionCommand(armSubsystem, 93)
                ),
                new SequentialCommandGroup( //Going up
                        new PositionCommand(armSubsystem, 93),
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .067, 115, 200),
                                new PositionCommand(wristSubsystem, 18)
                        )

                ),
                () -> elevatorSubsystem.getMechM() > .06
        );
    }

    public static Command scoreL3(){
        return new ConditionalCommand(
                new SequentialCommandGroup( //Going down
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .445, 40, 200),
                                new PositionCommand(wristSubsystem, 20)
                         ),
                        new PositionCommand(armSubsystem, 97.2)
                ),
                new SequentialCommandGroup( //Going up
                        new PositionCommand(armSubsystem, 97.2),
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .445, 200, 400),
                                new PositionCommand(wristSubsystem, 20)
                        )
                ),
                () -> elevatorSubsystem.getMechM() > .4
        );
    }

    
    public static Command scoreL4(){
        return new SequentialCommandGroup(
                new PositionCommand(armSubsystem, 99.5),
                new ParallelCommandGroup(
                        new PositionCommand(elevatorSubsystem, .905, 135, 400),
                        new PositionCommand(wristSubsystem, 27.76)
                )
        );
    }

    public static Command stable(){
        return new ConditionalCommand(
                new SequentialCommandGroup( //Won't clip elevator
                        new ParallelCommandGroup(
                                new PositionCommand(wristSubsystem, 0),
                                new PositionCommand(elevatorSubsystem, 0, 40, 200)
                        ),
                        new PositionCommand(armSubsystem, 0)
                ),
                new SequentialCommandGroup( //Will clip elevator
                        new PositionCommand(wristSubsystem, 0),
                        new PositionCommand(elevatorSubsystem, 0, 40, 200),
                        new PositionCommand(armSubsystem, 0)
                ),
                () -> wristSubsystem.getDegrees() < 50
        );
    }


    public static Command dunk(){
        return new SequentialCommandGroup(
       
        );
    }
    

}
