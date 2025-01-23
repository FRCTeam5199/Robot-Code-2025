package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.template.PositionCommand;

public class ScoreCommands {
    private static ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private static ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();
    private static WristSubsystem wristSubsystem = WristSubsystem.getInstance();
            
            
    public static Command intakeHP() {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        new PositionCommand(elevatorSubsystem, .13),
                        new PositionCommand(armSubsystem, 77),
                        new PositionCommand(wristSubsystem, 85)
                ),
                new SequentialCommandGroup(
                        new PositionCommand(armSubsystem, 77),
                        new PositionCommand(elevatorSubsystem, .13),
                        new PositionCommand(wristSubsystem, 85)
                ),
                () -> elevatorSubsystem.getMechM() > .13
        );
    }

    public static Command scoreL1(){
        return new SequentialCommandGroup(
  
        );
    }

    public static Command scoreL2(){
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        new PositionCommand(elevatorSubsystem, .067),
                        new PositionCommand(armSubsystem, 93),
                        new PositionCommand(wristSubsystem, 18)
                ),
                new SequentialCommandGroup(
                        new PositionCommand(armSubsystem, 93),
                        new PositionCommand(elevatorSubsystem, .067),
                        new PositionCommand(wristSubsystem, 18)
                ),
                () -> elevatorSubsystem.getMechM() > .067
        );
    }

    public static Command scoreL3(){
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        new PositionCommand(elevatorSubsystem, .445),
                        new PositionCommand(armSubsystem, 97.2),
                        new PositionCommand(wristSubsystem, 20)
                ),
                new SequentialCommandGroup(
                        new PositionCommand(armSubsystem, 97.2),
                        new PositionCommand(elevatorSubsystem, .445),
                        new PositionCommand(wristSubsystem, 20)
                ),
                () -> elevatorSubsystem.getMechM() > .445
        );
    }

    
    public static Command scoreL4(){
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        new PositionCommand(elevatorSubsystem, .905),
                        new PositionCommand(armSubsystem, 99.5),
                        new PositionCommand(wristSubsystem, 27.76)
                ),
                new SequentialCommandGroup(
                        new PositionCommand(armSubsystem, 99.5),
                        new PositionCommand(elevatorSubsystem, .905),
                        new PositionCommand(wristSubsystem, 27.76)
                ),
                () -> elevatorSubsystem.getMechM() > .905
        );
    }

    public static Command stable(){
        return new SequentialCommandGroup(
                new PositionCommand(wristSubsystem, 0),
                new PositionCommand(elevatorSubsystem, 0),
                new PositionCommand(armSubsystem, 0)
        );
    }


    public static Command dunk(){
        return new SequentialCommandGroup(
       
        );
    }
    

}
