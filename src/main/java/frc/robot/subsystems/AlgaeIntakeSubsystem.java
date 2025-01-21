package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.constants.Constants;
import frc.robot.subsystems.template.TemplateSubsystem;
import frc.robot.utility.FeedForward;
import frc.robot.utility.PID;
import frc.robot.utility.Type;

public class AlgaeIntakeSubsystem extends TemplateSubsystem {


    public AlgaeIntakeSubsystem() {
        super(Type.LINEAR,
                Constants.AlgaeIntakeConstants.ALGAEINTAKESUBYSTEM_ID,
                Constants.AlgaeIntakeConstants.ALGAEINTAKESUBYSTEM_CONSTRAINTS,
                Constants.AlgaeIntakeConstants.ALGAEINTAKESUBYSTEM_FEEDFORWARD,
                Constants.AlgaeIntakeConstants.ALGAEINTAKESUBYSTEM_lowerTOLERANCE,
                Constants.AlgaeIntakeConstants.ALGAEINTAKESUBYSTEM_upperTOLERANCE,
                Constants.AlgaeIntakeConstants.ALGAEINTAKESUBYSTEM_gearRatios,
                "Algae Intake");

//        configureMotor();
}
}