package frc.robot.tagalong;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;

public class ArmParser {
  ArmConfJson armConf;

  public PivotParser pivotParser;

  public ArmParser(File dir, String filename) {
    try {

      pivotParser = new PivotParser(
          new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/subsystems/armSubsystem/"),
          armConf.pivotFile
      );

    } catch (Exception err) {
      System.err.println(err);
      System.exit(1);
    }
  }
}
