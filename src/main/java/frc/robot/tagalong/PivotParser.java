package frc.robot.tagalong;

import java.io.File;

import com.fasterxml.jackson.databind.ObjectMapper;

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
