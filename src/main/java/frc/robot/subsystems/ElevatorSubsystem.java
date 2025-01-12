package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.Constants.ElevatorConstants;
import frc.robot.subsystems.template.TemplateSubsystem;
import frc.robot.utility.Type;

public class ElevatorSubsystem extends TemplateSubsystem {
    public ElevatorSubsystem(){
        super(Type.LINEAR, 
        ElevatorConstants.ELEVATOR_ID, 
        ElevatorConstants.ELEVATOR_CONSTRAINTS, 
        ElevatorConstants.ELEVATOR_PID, 
        ElevatorConstants.ELEVATOR_FF, 
        ElevatorConstants.ELEVATOR_MIN, 
        ElevatorConstants.ELEVATOR_MAX, 
        ElevatorConstants.ELEVATOR_GEARING);

        configureMotor(ElevatorConstants.INVERT, ElevatorConstants.BRAKE, ElevatorConstants.SUPPLY_CURRENT_LIMIT, ElevatorConstants.STATOR_CURRENT_LIMIT);
    }


    public Command setL1(){
        return new InstantCommand(()-> setPosition(0));
    
    }

    public Command setL2(){
        return new InstantCommand(()-> setPosition(0));

    }

    public Command setL3(){
        return new InstantCommand(()-> setPosition(0));

    }

    public Command setL4(){
        return new InstantCommand(()-> setPosition(0));

    }

    public Command setHP(){
        return new InstantCommand(()-> setPosition(0));

    }

    public Command setHeight(double position){
        return new InstantCommand(()-> setPosition(position));
    }


    


}