package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
<<<<<<< HEAD
import frc.robot.subsystems.template.AbstractSubsystem;
import frc.robot.utility.FeedForward;
import frc.robot.utility.PID;
import frc.robot.utility.SubsystemPrint;
import frc.robot.utility.Type;

public class IntakeSubsystem extends AbstractSubsystem {
  private static IntakeSubsystem intakeSubsystem;
  private static boolean subsystemOk = false;

  private IntakeSubsystem() {
    super(Type.ROLLER, new int[]{20, 21},
                new TrapezoidProfile.Constraints(90, 200),
                new PID(0, 0, 0), new FeedForward(0.23, 0.11904761904761904761904761904762),
                3, 3, new double[][]{{1, 1}, {1, 1}});
  }

  /** Gets the instance of this class, or creates one if one doesn't exsist. */
  public static IntakeSubsystem getInstance() {
    if (intakeSubsystem == null)
      intakeSubsystem = new IntakeSubsystem();
    return intakeSubsystem;
  }

  /**
   * The subsystem's initalize method. Only runs when the subsystem has been
   * initalized completely with no errors.
   */
  public void init() {
    try {
      configureMotors(false, false, new double[][]{{80, 80}, {80, 80}});

      new SubsystemPrint(this, "Initalized");
      subsystemOk = true;
    } catch (Exception e) {
      System.err.println(e.getStackTrace());
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (subsystemOk) subsystemPeriodic();
  }

  /**
   * The subsystem's periodic method. Only runs when the subsystem is ok with no
   * errors.
   */
  private void subsystemPeriodic() {
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
=======
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
>>>>>>> main
