package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.subsystems.template.TemplateSubsystem;
import frc.robot.utility.FeedForward;
import frc.robot.utility.PID;
import frc.robot.utility.Type;

public class ClimberSubsystem extends TemplateSubsystem {
    static ClimberSubsystem climber;
    
        public ClimberSubsystem() {
            super(Type.PIVOT,
                    Constants.ClimberConstants.CLIMBER_ID,
                    Constants.ClimberConstants.CLIMBER_CONSTRAINTS,
                    Constants.ClimberConstants.CLIMBER_FF,
                    Constants.ClimberConstants.CLIMBER_lowerTOLERANCE,
                    Constants.ClimberConstants.CLIMBER_upperTOLERANCE,
                    new double[][]{{125, 1}},
                    "Climber"
            );
    
            configureMotor(
                ClimberConstants.CLIMBER_INVERT,
                ClimberConstants.CLIMBER_BRAKE,
                ClimberConstants.CLIMBER_SUPPLY_CURRENT_LIMIT,
                ClimberConstants.CLIMBER_STATOR_CURRENT_LIMIT,
                ClimberConstants.CLIMBER_SLOT0_CONFIGS,
                ClimberConstants.CLIMBER_LOW_LIMIT,
                ClimberConstants.CLIMBER_HIGH_LIMIT
            );
        }
    
        public Command moveUP(){
            return new InstantCommand(()-> setPercent(30));
        }
    
        public Command moveDOWN(){
            return new InstantCommand(()-> setPercent(-10));
        }
    
    
        
        public static ClimberSubsystem getInstance() {
            if (climber == null) {
                climber = new ClimberSubsystem();
        }
        return climber;
    }
}