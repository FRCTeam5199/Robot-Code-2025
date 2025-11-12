package frc.robot.subsystems.logged.AprilTag;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Vision;
import frc.robot.subsystems.logged.AprilTag.AprilTagIO.AprilTagIOInputs;
import frc.robot.subsystems.logged.AprilTag.AprilTagIO.PoseObservation;

import org.littletonrobotics.junction.Logger;

public class NewAprilTagSubsystem extends SubsystemBase {
    private final VisionIOInputsAutoLogged[] inputs;
    private static NewAprilTagSubsystem aprilTagSubsystem;
    private final Alert[] disconnectedAlerts;
    private final AprilTagIO[] io;

    public static NewAprilTagSubsystem getInstance(){
        if (aprilTagSubsystem == null) {
            aprilTagSubsystem = new NewAprilTagSubsystem();
        }
        return aprilTagSubsystem;
    }
    public NewAprilTagSubsystem(AprilTagIO... io){
        this.io = io;

        // Initialize inputs
        this.inputs = new VisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < inputs.length; i++) {
        inputs[i] = new VisionIOInputsAutoLogged();
        }

        // Initialize disconnected alerts
        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < inputs.length; i++) {
        disconnectedAlerts[i] =
            new Alert(
                "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
        }
    }
    public boolean cameraPresent(String name){
        AprilTagIOInputs camera = null;
        for (AprilTagIOInputs input : inputs) {
            if (input.name == Vision.CAMERA_NAME) {
                camera = input;
            }
        }
        if (camera == null) {
            return false;
        }
        return camera.connected;
    }
    public Pair<Optional<EstimatedRobotPose>,Double> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var potentialPose : inputs[0].poseObservations) {
            if (potentialPose.ambiguity() > .1) continue;

            visionEst = photonEstimator.update(potentialPose);
            updateEstimationStdDevs(visionEst, potentialPose); 
            //TODO: is stddev updating supposed to be after we send the pose to the pose-estimator? 
            //TODO: additionally, consider giving more checks for clearly faulty positions (xyz within reasonable amounts).
        }
        if (visionEst.isPresent())
            return new Pair<>(visionEst, visionEst.get().timestampSeconds);
        else return new Pair<>(Optional.empty(), 0.0);
    }
    public static Matrix<N3, N1> curStdDevs = Constants.Vision.kTagStdDevs;
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, PoseObservation poseEstimate) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = Constants.Vision.kTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = Constants.Vision.kTagStdDevs;

            if (inputs[0].tagIds.length == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = Constants.Vision.kTagStdDevs;
            } else {
                poseEstimate.averageTagDistance();
                // Use default std devs if multiple targets are visible
                if (poseEstimate.tagCount() > 1) estStdDevs = Constants.Vision.kTagStdDevs;
                // Increase std devs based on distance
                else estStdDevs = estStdDevs.times(1 + (poseEstimate.averageTagDistance()*poseEstimate.averageTagDistance() / 30));
                curStdDevs = estStdDevs;
            }
        }
    }
    @Override
    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/Cameras/" + inputs[i].name, inputs[i]);
        }
        super.periodic();
    }
}
