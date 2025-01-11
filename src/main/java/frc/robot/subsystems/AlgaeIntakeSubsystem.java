package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.constants.Constants;
import frc.robot.subsystems.template.TemplateSubsystem;
import frc.robot.utils.FeedForward;
import frc.robot.utils.PID;
import frc.robot.utils.Type;

public class AlgaeIntakeSubsystem extends TemplateSubsystem {


    public AlgaeIntakeSubsystem(Type type, int id, TrapezoidProfile.Constraints constraints, PID pid, FeedForward feedForward, double lowerTolerance, double upperTolerance, double[][] gearRatios) {
        super(type, id, constraints, pid, feedForward, lowerTolerance, upperTolerance, gearRatios);
    }

    public AlgaeIntakeSubsystem() {
        super(Type.LINEAR,
                Constants.AlgaeIntakeConstants.ALGAEINTAKESUBYSTEM_ID,
                Constants.AlgaeIntakeConstants.ALGAEINTAKESUBYSTEM_CONSTRAINTS,
                Constants.AlgaeIntakeConstants.ALGAEINTAKESUBYSTEM_PID,
                Constants.AlgaeIntakeConstants.ALGAEINTAKESUBYSTEM_FEEDFORWARD,
                Constants.AlgaeIntakeConstants.ALGAEINTAKESUBYSTEM_lowerTOLERANCE,
                Constants.AlgaeIntakeConstants.ALGAEINTAKESUBYSTEM_upperTOLERANCE,
                Constants.AlgaeIntakeConstants.ALGAEINTAKESUBYSTEM_gearRatios);

//        configureMotor();
}
}