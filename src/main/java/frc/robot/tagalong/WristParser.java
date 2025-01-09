package frc.robot.tagalong;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.tagalong.ArmConfJson;
import frc.robot.tagalong.FileUtils;
import java.io.File;

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
