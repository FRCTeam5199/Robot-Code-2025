// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.SubsystemPrint;

public class ExampleSubsystem extends SubsystemBase {
  private static ExampleSubsystem exampleSubsystem;

  private static boolean subsystemInitalized = false;

  private ExampleSubsystem() {}

  /** Gets the instance of this class, or creates one if one doesn't exsist. */
  public static ExampleSubsystem getInstance() {
    if (exampleSubsystem == null) exampleSubsystem = new ExampleSubsystem();
    return exampleSubsystem;
  }

  /** The subsystem's initalize method. Only runs when the subsystem has been initalized completely with no errors. */
  public void init() {
    try {
      new SubsystemPrint(this, "Initalized");

      subsystemInitalized = true;
    } catch (Exception e) { System.err.println(e.getStackTrace()); }
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (subsystemInitalized) subsystemPeriodic();
  }

  /** The subsystem's periodic method. Only runs when the subsystem has been initalized completely with no errors. */
  private void subsystemPeriodic() {}

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
