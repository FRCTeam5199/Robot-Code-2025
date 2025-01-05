package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.template.AbstractSubsystem;
import frc.robot.subsystems.template.SubsystemType;
import frc.robot.utility.FeedForward;
import frc.robot.utility.PID;
import frc.robot.utility.SubsystemPrint;

public class IntakeSubsystem extends AbstractSubsystem {
  private static IntakeSubsystem intakeSubsystem;
  private static boolean subsystemOk = false;

  private IntakeSubsystem() {
    super(SubsystemType.ROLLER, 25,
        new TrapezoidProfile.Constraints(90, 200),
        new PID(0, 0, 0), new FeedForward(.23, .11904761904761904761904761904762),
        3, 3, new double[][] { { 1, 1 } });
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
      configureMotor(true, false, 80, 80);

      new SubsystemPrint(this, "Initalized");
      subsystemOk = true;
    } catch (Exception e) {
      System.err.println(e.getStackTrace());
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (subsystemOk)
      subsystemPeriodic();
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
