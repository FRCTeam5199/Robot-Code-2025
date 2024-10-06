package frc.robot.utility;

public class SubsystemPrint {
  public SubsystemPrint(Object messageOriginName, String message) {
    System.out.println(messageOriginName.getClass().getName() + ": " + message);
  }
}
