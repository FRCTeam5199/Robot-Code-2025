package frc.robot.tagalong;

import com.fasterxml.jackson.databind.ObjectMapper;

import frc.robot.tagalong.FileUtils;
import frc.robot.tagalong.PivotConfJson;

import java.io.File;

public class PivotParser {
  public PivotConfJson pivotConf;
  public PivotParser(File dir, String filename) {
    try {
      File pivotFile = new File(dir, filename);
      FileUtils.checkForFile(pivotFile);
      pivotConf = new ObjectMapper().readValue(pivotFile, PivotConfJson.class);

    } catch (Exception err) {
      System.err.println(err);
      System.exit(1);
    }
  }
}
