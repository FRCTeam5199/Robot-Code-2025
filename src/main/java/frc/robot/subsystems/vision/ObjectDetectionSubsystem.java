package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.constants.Constants;

public class ObjectDetectionSubsystem extends SubsystemBase {
    private static ObjectDetectionSubsystem objectDetectionSubsystem;

    public static ObjectDetectionSubsystem getInstance() {
        if (objectDetectionSubsystem == null) objectDetectionSubsystem = new ObjectDetectionSubsystem();
        return objectDetectionSubsystem;
    }

    /**
     * Gets the distance of the object from the robot.
     * @return
     */
    public String getObjectClass(){
        return LimelightHelpers.getClassifierClass(Constants.Vision.Limelight.LIMELIGHT_NAME);
    }

    /**
     * Gets the horizontal position of the object from the center of the screen
     * @return
     */
    public double getObjectXPosition(){
        return LimelightHelpers.getTX(Constants.Vision.Limelight.LIMELIGHT_NAME);
    }

    /**
     * Gets the area of the object on the screen
     * @return
     */
    public double getObjectArea(){
        return LimelightHelpers.getTA(Constants.Vision.Limelight.LIMELIGHT_NAME);
    }
}