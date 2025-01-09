package frc.robot.tagalong;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.tagalong.ArmConfJson;
import frc.robot.tagalong.FileUtils;
import java.io.File;

public class ArmParser {
  ArmConfJson armConf;

  public PivotParser pivotParser;

  public ArmParser(File dir, String filename) {
    try {

      pivotParser = new PivotParser(
          new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/configs/arm"),
          armConf.pivotFile
      );

    } catch (Exception err) {
      System.err.println(err);
      System.exit(1);
    }
  }
}
