package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Classifier;
import frc.robot.LimelightHelpers.LimelightTarget_Detector;

public class ObjectDetectionSubsystem extends SubsystemBase {
    private static ObjectDetectionSubsystem objectDetectionSubsystem;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tclass = table.getEntry("tclass");

    //read values periodically
    double x;
    double area;
    double y;


    @Override
    public void periodic() {
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.00);

    }

    String cwass = tclass.getString("no piece");

    public static ObjectDetectionSubsystem getInstance() {
        if (objectDetectionSubsystem == null) objectDetectionSubsystem = new ObjectDetectionSubsystem();
        return objectDetectionSubsystem;
    }


    public LimelightResults getLimelightResults() {
        return LimelightHelpers.getLatestResults(Constants.VisionConstants.LIMELIGHT_NAME); //Why is there a LIMELIGHT here?
    }


    public LimelightTarget_Classifier[] getTargets() {
        return getLimelightResults().targets_Classifier;
    }

    public LimelightTarget_Detector[] getObjects() {
        return getLimelightResults().targets_Detector;

    }

    public LimelightTarget_Detector getNearestAlgae() {
        LimelightTarget_Detector nearestAlgae = null;
        double minDistance = Double.MAX_VALUE;

        for (LimelightTarget_Detector object : getObjects()) {
            if (object.className.equals("Algae")) {
                double distance = calculateDistance(object); // Implement this method
                if (distance < minDistance) {
                    nearestAlgae = object;
                    minDistance = distance;
                }
            }
        }

        return nearestAlgae;
    }

    private double calculateDistance(LimelightTarget_Detector object) {
        // Calculate the distance to the object based on its coordinates (tx, ty)
        // Replace this with the actual distance calculation based on your needs
        return Math.sqrt(object.tx * object.tx + object.ty * object.ty);
    }

    public double getAlgaePoseX() {
        return x;
    }

    public double getAlgaePoseY() {
        return y;
    }

    public double getAlgaeDistance() {
        return area;
    }

    public double getObjectIdentity() {
        return x;
    }

    public boolean algaePresent() {
        if (x == 0 && y == 0 && area == 0) {
            return false;
        } else {
            return true;
        }
    }


}
