package frc.robot.tagalong;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;

public class WristParser {
  WristConfJson wristConf;

  public PivotParser pivotParser;

  public WristParser(File dir, String filename) {
    try {

      pivotParser = new PivotParser(
          new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/configs/subsystems/wrist"),
          wristConf.pivotFile
      );

    } catch (Exception err) {
      System.err.println(err);
      System.exit(1);
    }
  }
}
