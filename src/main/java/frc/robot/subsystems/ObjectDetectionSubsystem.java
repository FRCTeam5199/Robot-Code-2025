package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;

public class ObjectDetectionSubsystem extends SubsystemBase {
    private static ObjectDetectionSubsystem objectDetectionSubsystem;

    private NetworkTable networkTable = NetworkTableInstance.getDefault().getTable(Constants.PhotonVisionConstants.CAMERA_NAME);
    private NetworkTableEntry targetX = networkTable.getEntry("tx");
    private NetworkTableEntry targetY = networkTable.getEntry("ty");
    private NetworkTableEntry targetArea = networkTable.getEntry("ta");
    private NetworkTableEntry targetClass = networkTable.getEntry("tclass");

    public static ObjectDetectionSubsystem getInstance() {
        if (objectDetectionSubsystem == null) objectDetectionSubsystem = new ObjectDetectionSubsystem();
        return objectDetectionSubsystem;
    }

    public boolean isObjectDetected() {
        return targetArea.getDouble(0) != 0;
    }

    /**
     * Gets the distance of the object from the robot.
     * @return
     */
    public String getObjectClass(){
        return targetClass.getString("");
    }

    /**
     * Gets the distance of the object from the robot.
     * @return
     */
    public double getObjectDistance(){
        return Math.sqrt(Math.pow(targetX.getDouble(0), 2) + Math.pow(targetY.getDouble(0), 2));
    }

    /**
     * Gets a Pose3d of the object relative to the robot
     * @return
     */
    public Pose3d getObjectPositionRobotRelative(){
        return LimelightHelpers.getTargetPose3d_RobotSpace(Constants.PhotonVisionConstants.CAMERA_NAME);
    }

    /**
     * Gets a Pose2d of the object relative to the robot
     * @return
     */
    public Pose2d getObjectPositionFieldRelative(){
        return new Pose2d(RobotContainer.commandSwerveDrivetrain.getPose().getX() + this.getObjectPositionRobotRelative().getX(),
                        RobotContainer.commandSwerveDrivetrain.getPose().getY() + this.getObjectPositionRobotRelative().getY(),
                        new Rotation2d(RobotContainer.commandSwerveDrivetrain.getPose().getRotation().getRadians() + this.getObjectPositionRobotRelative().getRotation().toRotation2d().getRadians()));
    }
}
