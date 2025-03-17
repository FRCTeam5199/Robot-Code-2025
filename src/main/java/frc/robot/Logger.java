package frc.robot;

import java.io.FileWriter;
import java.io.IOException;
import java.time.Instant;
import java.util.Date;

import edu.wpi.first.wpilibj.Filesystem;

public class Logger {
    FileWriter logFile;
    private Logger() {}

    public void createLogFile() {
        try {
            logFile = new FileWriter(Filesystem.getDeployDirectory() + "logs/" + Date.from(Instant.now()));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void writeToLogFile(String data) {
        try {
            logFile.write(data);
        } catch (IOException e) { e.printStackTrace(); }
    }
}
