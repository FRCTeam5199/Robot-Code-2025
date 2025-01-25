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
                                new PositionCommand(elevatorSubsystem, .151, 40, 200),
                                new PositionCommand(wristSubsystem, -0.028)
                        ),
                        new PositionCommand(armSubsystem, 86.043)
                ),
                new SequentialCommandGroup( //Going up
                        new PositionCommand(armSubsystem, 86.043),
                        new PositionCommand(elevatorSubsystem, .151, 115, 200),
                        new PositionCommand(wristSubsystem, 0.028)
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
                                new PositionCommand(elevatorSubsystem,  0., 40, 200),
                                new PositionCommand(wristSubsystem, 66.837) //30
                         ),
                        new PositionCommand(armSubsystem, 87.21)
                ),
                new SequentialCommandGroup( //Going up
                        new PositionCommand(armSubsystem, 87.21),
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem,  0.45, 200, 400),
                                new PositionCommand(wristSubsystem, 66.837) //30
                        )
                ),
                () -> elevatorSubsystem.getMechM() > .4
        );
    }

    
    public static Command scoreL4(){
        return new SequentialCommandGroup(
                new PositionCommand(armSubsystem, 94.59),
                new ParallelCommandGroup(
                        new PositionCommand(elevatorSubsystem, 0.947, 135, 400),
                        new PositionCommand(wristSubsystem, 63.32)//27.76
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
                        new PositionCommand(armSubsystem, 10)
                ),
                new SequentialCommandGroup( //Will clip elevator
                        new PositionCommand(wristSubsystem, 0),
                        new PositionCommand(elevatorSubsystem, 0, 40, 200),
                        new PositionCommand(armSubsystem, 10)
                ),
                () -> wristSubsystem.getDegrees() < 50
        );
    }


    public static Command dunk(){
        return new SequentialCommandGroup(
       
        );
    }
    

}
