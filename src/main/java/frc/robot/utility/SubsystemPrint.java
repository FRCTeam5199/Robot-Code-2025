package frc.robot.utility;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SubsystemPrint {
  public SubsystemPrint(SubsystemBase subsystem, String message) {
    System.out.println(subsystem.getName() + ": " + message);
  }
}
