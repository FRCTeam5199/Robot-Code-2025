package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ScoreCommands {
    static ArmSubsystem arm;
    static ElevatorSubsystem elevator;
    static WristSubsystem wrist;
            
            
    public static Command scoreHP(){
        return new SequentialCommandGroup(
   
        );
    }

    public static Command scoreL1(){
        return new SequentialCommandGroup(
  
        );
    }

    public static Command scoreL2(){
        return new SequentialCommandGroup(
 
        );
    }

    public static Command scoreL3(){
        return new SequentialCommandGroup(

        );
    }

    
    public static Command scoreL4(){
        return new SequentialCommandGroup(
        
        );
    }

    public static Command dunk(){
        return new SequentialCommandGroup(
       
        );
    }
    

}
