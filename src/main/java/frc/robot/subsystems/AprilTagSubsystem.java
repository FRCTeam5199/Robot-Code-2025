package frc.robot.subsystems;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.tagalong.Controlboard;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
    private Matrix<N3, N1> curStdDevs;
    private AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    public static AprilTagSubsystem aprilTagSubsystem;


    public AprilTagSubsystem() {
        camera = new PhotonCamera(Constants.Vision.CAMERA_NAME);

        photonEstimator =
                new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.Vision.CAMERA_POSE);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);


    }

    @Override
    public void periodic() {
//        System.out.println("X: " + getViewedTargets().getFirst() + " Y: " + getViewedTargets().getSecond());
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
     * {@link getEstimationStdDevs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     * used for estimation.
     */
    public Pair<Optional<EstimatedRobotPose>, Double> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : camera.getAllUnreadResults()) {
            visionEst = photonEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());


        }
        if (visionEst.isPresent()) return new Pair<>(visionEst, visionEst.get().timestampSeconds);
        else return new Pair<>(Optional.empty(), 0.0);
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets       All targets in this camera frame
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = Constants.Vision.kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = Constants.Vision.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = Constants.Vision.kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = Constants.Vision.kSingleTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }


    public Pair<Double, Double> getViewedTargets() {
        double x = 0;
        double y = 0;

        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);

            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                var tagpose = photonEstimator.getFieldTags().getTagPose(result.getBestTarget().getFiducialId());
                x = tagpose.get().getX();
                y = tagpose.get().getY();
            }
        }
        return new Pair<Double, Double>(x, y);

    }


    public static AprilTagSubsystem getInstance() {
        if (aprilTagSubsystem == null) {
            aprilTagSubsystem = new AprilTagSubsystem();
        }
        return aprilTagSubsystem;
    }


}