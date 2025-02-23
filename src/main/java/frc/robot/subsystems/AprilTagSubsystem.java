package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import frc.robot.RobotContainer;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Vision;

public class AprilTagSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
    private Matrix<N3, N1> curStdDevs;
    private AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    private double closestTagX = 0, closestTagY = 0, closestTagYaw = 0;

    private boolean isAutoAligning = false;

    public int getClosestTagID() {
        return closestTagID;
    }

    private int closestTagID = -1;
    private List<PhotonPipelineResult> results = new ArrayList<>();

    private static AprilTagSubsystem aprilTagSubsystem;
    private static CommandSwerveDrivetrain commandSwerveDrivetrain = RobotContainer.commandSwerveDrivetrain;

    double[] tagAngles = {1, 1, 1, 1, 1, 1, 300, 0, 60, 120, 180, 240, 1, 1, 1, 1, 1, 240, 180, 120, 60, 0, 300};

    public AprilTagSubsystem() {
        camera = new PhotonCamera(Constants.Vision.CAMERA_NAME);

        photonEstimator =
                new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.Vision.CAMERA_POSE);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);


    }

    @Override
    public void periodic() {
//        System.out.println("Drive rotation: " + commandSwerveDrivetrain.getPose().getRotation().getDegrees());
//        System.out.println("Pigeon angle: " + commandSwerveDrivetrain.getPigeon2().getRotation2d().getDegrees());
//        System.out.println("Closest tag id: " + closestTagID);
        results = camera.getAllUnreadResults();
        updateClosestTagID();
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
        for (var change : results) {
            visionEst = photonEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());
        }
        if (visionEst.isPresent() && !isAutoAligning)
            return new Pair<>(visionEst, visionEst.get().timestampSeconds);
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
                if (numTags > 1) estStdDevs = Constants.Vision.kMultiTagStdDevs;
                    // Increase std devs based on (average) distance
//                if (numTags == 1 && avgDist > 4)
//                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
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

    public void setStdDevs(Matrix<N3, N1> stdDevs) {
        curStdDevs = stdDevs;
    }

    public int updateClosestTagID() {
        PhotonTrackedTarget bestTarget = null;
        if (!results.isEmpty()) {
            PhotonPipelineResult result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                double smallestDistance = Double.MAX_VALUE;

                for (PhotonTrackedTarget target : result.getTargets()) {
                    if ((target.getFiducialId() >= 6 && target.getFiducialId() <= 11)
                            || (target.getFiducialId() >= 17 && target.getFiducialId() <= 22)) {
                        double distance = PhotonUtils.calculateDistanceToTargetMeters(
                                Constants.Vision.CAMERA_POSE.getZ(), .31,
                                Constants.Vision.CAMERA_POSE.getRotation().getY(),
                                Units.degreesToRadians(target.getPitch()));

                        if (distance < smallestDistance) {
                            bestTarget = target;
                            smallestDistance = distance;
                        }
                    }
                }
            }
        }

        if (bestTarget != null) closestTagID = bestTarget.getFiducialId();
        return closestTagID;
    }

    public double getRotationToAlign(int id) {
        if (id == -1) return 0;
        return -180 - tagAngles[id] + commandSwerveDrivetrain.getPose().getRotation().getDegrees();
    }

    public double getRotationToAlign() {
        if (closestTagID == -1) return 0;
        return -180 - tagAngles[closestTagID] + commandSwerveDrivetrain.getPose().getRotation().getDegrees();
    }
//
//    public double getTargetHeight(PhotonTrackedTarget target) {
//        double tag = target.getFiducialId();
//        double height = 1;
//        for (int i = 0; i < tagAngles.length; i++) {
//            if (tag == i) {
//                height = tagAngles[i];
//            }
//        }
//        return height;
//    }
//
//    public int closestTarget() {
//        Pair<Double, Integer> closest = new Pair(100000000, 0);
//        Pair<Double, Integer> closest2 = new Pair(100000, 0);
//
//
//        for (int i = 0; i < results.size(); i++) {
//            for (int z = 0; i < results.get(i).getTargets().size(); z++) {
//                if (getTargetAngle(results.get(i).getTargets().get(z)) != 1)
//                    closest2 = new Pair(PhotonUtils.calculateDistanceToTargetMeters(Vision.CAMERA_POSE.getZ(), .308, Units.degreesToRadians(Vision.CAMERA_POSE.getRotation().getMeasureY().baseUnitMagnitude()), getTargetAngle(results.get(i).getTargets().get(z))), results.get(i).getTargets().get(z).getFiducialId());
//            }
//            if (closest2.getFirst() < closest.getFirst()) {
//                closest = closest2;
//            }
//        }
//        return closest.getSecond();
//    }
//
//    public List<Double> alignValues(int tag) {
//        double x = 80, y = 80, z = 80;
//        List<Double> targetValue = new ArrayList<Double>();
//        targetValue.add(x);
//        targetValue.add(y);
//        targetValue.add(z);
//
//        if (tag != 0 || tag != -1) {
//            for (int i = 0; i < results.size(); i++) {
//                for (int p = 0; p < results.size(); p++) {
//                    if (results.get(i).getTargets().get(p).getFiducialId() == tag) {
//                        x = results.get(i).getTargets().get(p).getPitch();
//                        y = results.get(i).getTargets().get(p).getYaw();
//                        z = getTargetAngle(results.get(i).getTargets().get(p)) + 180;
//
//                    }
//                }
//            }
//        }
//
//        return targetValue;
//
//    }

    public double[] getClosestTagXYYaw() {
        if (!results.isEmpty()) {
            PhotonPipelineResult result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                double smallestDistance = Double.MAX_VALUE;
                PhotonTrackedTarget bestTarget = null;

                for (PhotonTrackedTarget target : result.getTargets()) {
                    if ((target.getFiducialId() >= 6 && target.getFiducialId() <= 11)
                            || (target.getFiducialId() >= 17 && target.getFiducialId() <= 22)) {
                        double distance = PhotonUtils.calculateDistanceToTargetMeters(
                                Constants.Vision.CAMERA_POSE.getZ(), .31,
                                Constants.Vision.CAMERA_POSE.getRotation().getY(),
                                Units.degreesToRadians(target.getPitch()));

                        if (distance < smallestDistance) {
                            bestTarget = target;
                            smallestDistance = distance;
                        }
                    }
                }
                if (bestTarget == null) {
                    return new double[]{0.0, 0.0, 0.0};
                }


                closestTagID = bestTarget.getFiducialId();

                closestTagX = -(Math.cos(Math.toRadians(bestTarget.getYaw())) * smallestDistance);
                closestTagY = Math.sin(Math.toRadians(bestTarget.getYaw())) * smallestDistance;
                closestTagID = bestTarget.getFiducialId();

                closestTagX += Vision.CAMERA_TO_FRONT_DISTANCE;

                closestTagYaw = bestTarget.getYaw();

//                System.out.println("Id: " + bestTarget.getFiducialId()
//                        + " X: " + closestTagX + " Y: " + closestTagY);
            }
        }
        return new double[]{closestTagX, closestTagY, closestTagYaw};
    }


    public static AprilTagSubsystem getInstance() {
        if (aprilTagSubsystem == null) {
            aprilTagSubsystem = new AprilTagSubsystem();
        }
        return aprilTagSubsystem;
    }

    public boolean isAutoAligning() {
        return isAutoAligning;
    }

    public void setAutoAligning(boolean autoAligning) {
        isAutoAligning = autoAligning;
    }
}