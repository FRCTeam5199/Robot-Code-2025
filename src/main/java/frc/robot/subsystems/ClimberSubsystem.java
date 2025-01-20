package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.constants.Constants;
import frc.robot.subsystems.template.TemplateSubsystem;
import frc.robot.utility.FeedForward;
import frc.robot.utility.PID;
import frc.robot.utility.Type;

public class ClimberSubsystem extends TemplateSubsystem {

    public ClimberSubsystem() {
        super(Type.PIVOT,
                Constants.ClimberConstants.CLIMBER_ID,
                Constants.ClimberConstants.CLIMBER_CONSTRAINTS,
                Constants.ClimberConstants.CLIMBER_FEEDFORWARD,
                Constants.ClimberConstants.CLIMBER_lowerTOLERANCE,
                Constants.ClimberConstants.CLIMBER_upperTOLERANCE,
                Constants.ClimberConstants.CLIMBER_gearRatios);
    }

}