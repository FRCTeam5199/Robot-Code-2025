package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.constants.Constants;
import frc.robot.subsystems.TemplateSubsystem;
import frc.robot.utils.FeedForward;
import frc.robot.utils.PID;
import frc.robot.utils.Type;

public class ClawSubsystem extends TemplateSubsystem {
    public ClawSubsystem(Type type, int id, TrapezoidProfile.Constraints constraints, PID pid, FeedForward feedForward, double lowerTolerance, double upperTolerance, double[][] gearRatios) {
        super(type, id, constraints, pid, feedForward, lowerTolerance, upperTolerance, gearRatios);
    }

    public ClawSubsystem(){
        super(Type.LINEAR,
                Constants.ClawConstants.CLAW_ID,
                Constants.ClawConstants.CLAW_CONSTRAINTS,
                Constants.ClawConstants.CLAW_PID,
                Constants.ClawConstants.CLAW_FEEDFORWARD,
                Constants.ClawConstants.CLAW_lowerTOLERANCE,
                Constants.ClawConstants.CLAW_upperTOLERANCE,
                Constants.ClawConstants.CLAW_gearRatios);

//        configureMotor(Constants.ElevatorConstants.INVERT, Constants.ElevatorConstants.BRAKE, Constants.ElevatorConstants.SUPPLY_CURRENT_LIMIT, Constants.ElevatorConstants.STATOR_CURRENT_LIMIT);
    }
}