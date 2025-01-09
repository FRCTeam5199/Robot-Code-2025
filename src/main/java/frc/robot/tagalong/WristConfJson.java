package frc.robot.tagalong;


public class WristConfJson extends TagalongBaseJson {
  public String name;
  public String flywheelLeftFile;
  public String flywheelRightFile;
  public String pivotFile;
  public UnitJson pivotHeight;
  public String pivotConflict;
  public PositionalLimitsJson pivotUnsafePositionalLimits;
  public PolynomialJson targetPivotQuadratic = PolynomialJson.getDefaultRotation();
  public PolynomialJson minPivotQuadratic = PolynomialJson.getDefaultRotation();
  public PolynomialJson maxPivotQuadratic = PolynomialJson.getDefaultRotation();
  public LinearizedJson targetPivotLinearized;
  public LinearizedJson minPivotLinearized;
  public LinearizedJson maxPivotLinearized;
  public LinearizedJson launchRPSLinearized;
  public int breakBeamChannel;
}
