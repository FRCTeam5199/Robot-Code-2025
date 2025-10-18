package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.RobotContainer;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Vision;

public class AprilTagSubsystem extends SubsystemBase {
    public final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;

    public final PhotonCamera backCamera;
    private final PhotonPoseEstimator backPhotonEstimator;

    private Matrix<N3, N1> curStdDevs;
    private AprilTagFieldLayout kTagLayout;
    private double closestTagX = 0, closestTagY = 0, closestTagYaw = 0;
    private double closestTagXBack = 0, closestTagYBack = 0, closestTagYawBack = 0;
    private boolean doneAutoDriving = false;

    private int closestTagID = -1;
    private int backClosestTagID = -1;
    private List<PhotonPipelineResult> results = new ArrayList<>();
    private List<PhotonPipelineResult> backResults = new ArrayList<>();

    private static AprilTagSubsystem aprilTagSubsystem;
    private static CommandSwerveDrivetrain commandSwerveDrivetrain = RobotContainer.commandSwerveDrivetrain;

    double[] tagAngles = {1, 1, 1, 1, 1, 1, 300, 0, 60, 120, 180, 240, 1, 1, 1, 1, 1, 240, 180, 120, 60, 0, 300};

    private static ArrayList<Pair<Double, Double>> lookUpTable = new ArrayList<>() {
        {
            //distance, stddev
            add(new Pair<>(.5842 + Vision.CAMERA_TO_FRONT_DISTANCE, .25));
            add(new Pair<>(1.3843 + Vision.CAMERA_TO_FRONT_DISTANCE, 1.25d));
            add(new Pair<>(2.8194 + Vision.CAMERA_TO_FRONT_DISTANCE, 3d));
        }
    };

    //4, 5, 14, 15
    public AprilTagSubsystem() {
        camera = new PhotonCamera(Constants.Vision.CAMERA_NAME);

        try {
            kTagLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory()
                    + "/apriltagLayout.json");
        } catch (Exception ignored) {
        }

        photonEstimator =
                new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.Vision.CAMERA_POSE);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);


        backCamera = new PhotonCamera(Vision.BACK_CAMERA_NAME);

        backPhotonEstimator =
                new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Vision.BACK_CAMERA_POSE);
        backPhotonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void periodic() {
//        System.out.println("Drive rotation: " + commandSwerveDrivetrain.getPose().getRotation().getDegrees());
//        System.out.println("Pigeon angle: " + commandSwerveDrivetrain.getPigeon2().getRotation2d().getDegrees());
//        System.out.println("Closest tag id: " + closestTagID);
        results = camera.getAllUnreadResults();
        backResults = backCamera.getAllUnreadResults();
        updateClosestTagID();
        updateBackClosestTagID();
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     * used for estimation.
     */
    public Pair<Optional<EstimatedRobotPose>, Double> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : results) {
            boolean shouldContinue = false;
            for (PhotonTrackedTarget target : change.getTargets())
                if (target.getPoseAmbiguity() > .1) shouldContinue = true;
            if (shouldContinue) continue;

            visionEst = photonEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());
        }
        if (visionEst.isPresent())
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
            curStdDevs = Constants.Vision.kTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = Constants.Vision.kTagStdDevs;
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
                curStdDevs = Constants.Vision.kTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = Constants.Vision.kTagStdDevs;
                    // Increase std devs based on (average) distance
//                if (numTags == 1 && avgDist > 4)
//                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }


    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     * used for estimation.
     */
    public Pair<Optional<EstimatedRobotPose>, Double> getBackEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : backResults) {
            boolean shouldContinue = false;
            for (PhotonTrackedTarget target : change.getTargets())
                if (target.getPoseAmbiguity() > .1) shouldContinue = true;
            if (shouldContinue) continue;

            visionEst = backPhotonEstimator.update(change);
            updateBackEstimationStdDevs(visionEst, change.getTargets());
        }
        if (visionEst.isPresent())
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
    private void updateBackEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = Constants.Vision.kTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = Constants.Vision.kTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = backPhotonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
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
                curStdDevs = Constants.Vision.kTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = Constants.Vision.kTagStdDevs;
                    // Increase std devs based on (average) distance
//                if (numTags == 1 && avgDist > 4)
//                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

//    public Matrix<N3, N1> getStdDevs(double distance) {
//        double stdDev;
//        //Checks for if the distance is outside the bounds of the look up table
//        if (distance < lookUpTable.get(0).getFirst()) {
//            stdDev = lookUpTable.get(0).getSecond();
//            return VecBuilder.fill(stdDev, stdDev, 999);
//        }
//        if (distance > lookUpTable.get(lookUpTable.size() - 1).getFirst()) {
//            stdDev = lookUpTable.get(lookUpTable.size() - 1).getSecond();
//            return VecBuilder.fill(stdDev, stdDev, 999);
//        }
//
//        //Interpolation
//        double lowDistance = lookUpTable.get(0).getFirst();
//        double highDistance = lookUpTable.get(lookUpTable.size() - 1).getFirst();
//        int lowIndex = 0;
//        int highIndex = lookUpTable.size() - 1;
//        for (int i = 0; i < lookUpTable.size() - 1; i++) {
//            if (distance > lookUpTable.get(i).getFirst()
//                    && distance < lookUpTable.get(i + 1).getFirst()) {
//                lowDistance = lookUpTable.get(i).getFirst();
//                highDistance = lookUpTable.get(i + 1).getFirst();
//                lowIndex = i;
//                highIndex = i + 1;
//                break;
//            }
//        }
//
//        double percentInBetween = (distance - lowDistance) / (highDistance - lowDistance);
//        stdDev = (percentInBetween *
//                (lookUpTable.get(highIndex).getSecond() - lookUpTable.get(lowIndex).getSecond())) +
//                lookUpTable.get(lowIndex).getSecond();
//
//        return VecBuilder.fill(stdDev, stdDev, 999);
//    }

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

    public boolean
    cameraPresent() {
        return camera.isConnected();
    }

    public int updateClosestTagID() {
        PhotonTrackedTarget bestTarget = null;
        if (!results.isEmpty()) {
            PhotonPipelineResult result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                double smallestAngleChange = doneAutoDriving ? 45 : 25;

                for (PhotonTrackedTarget target : result.getTargets()) {
                    if ((target.getFiducialId() >= 6 && target.getFiducialId() <= 11) //ignores tags not on the reef
                            || (target.getFiducialId() >= 17 && target.getFiducialId() <= 22)) {
                        double pigeonAngle = commandSwerveDrivetrain.getPigeon2().getRotation2d().getDegrees();

                        while (pigeonAngle > 360) pigeonAngle -= 360;
                        while (pigeonAngle < -360) pigeonAngle += 360;

                        if (pigeonAngle < 0) pigeonAngle = 360 + pigeonAngle;

                        double angleChange = Math.abs((-180 - tagAngles[target.getFiducialId()]) + pigeonAngle);

                        while (angleChange > 180) angleChange = Math.abs(angleChange - 360);

                        if (angleChange < smallestAngleChange) {
                            smallestAngleChange = angleChange;
                            bestTarget = target;
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
        double rotation = -180 - tagAngles[id] + commandSwerveDrivetrain.getPose().getRotation().getDegrees();
        return RobotContainer.getShouldAlignBackwards() ? rotation + 180 : rotation;
    }

    public double[] getClosestTagXYYaw() {
        if (!results.isEmpty()) {
            PhotonPipelineResult result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                double smallestAngleChange = doneAutoDriving ? 45 : 25;
                PhotonTrackedTarget bestTarget = null;

                for (PhotonTrackedTarget target : result.getTargets()) {
                    if ((target.getFiducialId() >= 6 && target.getFiducialId() <= 11) //ignores tags not on the reef
                            || (target.getFiducialId() >= 17 && target.getFiducialId() <= 22)) {
                        double pigeonAngle = commandSwerveDrivetrain.getPigeon2().getRotation2d().getDegrees();

                        while (pigeonAngle > 360) pigeonAngle -= 360;
                        while (pigeonAngle < -360) pigeonAngle += 360;

                        if (pigeonAngle < 0) pigeonAngle = 360 + pigeonAngle;

                        double angleChange = Math.abs((-180 - tagAngles[target.getFiducialId()]) + pigeonAngle);
                        if (DriverStation.getAlliance().isPresent()
                                && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red))
                            angleChange += 180;

                        while (angleChange > 180) angleChange = Math.abs(angleChange - 360);
                        // System.out.println("Angle Change: " + angleChange);

                        if (angleChange < smallestAngleChange) {
                            smallestAngleChange = angleChange;
                            bestTarget = target;
                        }
                    }
                }
                if (bestTarget == null) {
                    return new double[]{0.0, 0.0, 0.0};
                }
                closestTagID = bestTarget.getFiducialId();

                double smallestDistance = PhotonUtils.calculateDistanceToTargetMeters(
                        Constants.Vision.CAMERA_POSE.getZ(), .31,
                        Constants.Vision.CAMERA_POSE.getRotation().getY(),
                        Units.degreesToRadians(bestTarget.getPitch()));

                closestTagX = -(Math.cos(Math.toRadians(bestTarget.getYaw())) * smallestDistance);
                closestTagY = Math.sin(Math.toRadians(bestTarget.getYaw())) * smallestDistance;

                closestTagX += Vision.CAMERA_TO_FRONT_DISTANCE;

                closestTagYaw = bestTarget.getYaw();

                if (DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
                    closestTagX = -closestTagX;
                    closestTagY = -closestTagY;
                }

//                System.out.println("Id: " + bestTarget.getFiducialId()
//                        + " X: " + closestTagX + " Y: " + closestTagY);
            }
        }
        return new double[]{closestTagX, closestTagY, closestTagYaw};
    }

    public int updateBackClosestTagID() {
        PhotonTrackedTarget bestTarget = null;
        if (!backResults.isEmpty()) {
            PhotonPipelineResult result = backResults.get(backResults.size() - 1);
            if (result.hasTargets()) {
                double smallestAngleChange = doneAutoDriving ? 45 : 25;

                for (PhotonTrackedTarget target : result.getTargets()) {
                    if ((target.getFiducialId() >= 6 && target.getFiducialId() <= 11) //ignores tags not on the reef
                            || (target.getFiducialId() >= 17 && target.getFiducialId() <= 22)) {
                        double pigeonAngle = commandSwerveDrivetrain.getPigeon2().getRotation2d().getDegrees();

                        while (pigeonAngle > 360) pigeonAngle -= 360;
                        while (pigeonAngle < -360) pigeonAngle += 360;

                        if (pigeonAngle < 0) pigeonAngle = 360 + pigeonAngle;

                        double angleChange = Math.abs((-180 - tagAngles[target.getFiducialId()]) + pigeonAngle);

                        while (angleChange > 180) angleChange = Math.abs(angleChange - 360);

                        if (angleChange < smallestAngleChange) {
                            smallestAngleChange = angleChange;
                            bestTarget = target;
                        }
                    }
                }
            }
        }

        if (bestTarget != null) backClosestTagID = bestTarget.getFiducialId();
        return closestTagID;
    }

    public double[] getBackClosestTagXYYaw() {
        if (!backResults.isEmpty()) {
            PhotonPipelineResult result = backResults.get(backResults.size() - 1);
            if (result.hasTargets()) {
//                double smallestAngleChange = 20;
                double smallestAngleChange = 999;
                PhotonTrackedTarget bestTarget = null;

                for (PhotonTrackedTarget target : result.getTargets()) {
                    if ((target.getFiducialId() >= 6 && target.getFiducialId() <= 11) //ignores tags not on the reef
                            || (target.getFiducialId() >= 17 && target.getFiducialId() <= 22)) {
                        double pigeonAngle = commandSwerveDrivetrain.getPigeon2().getRotation2d().getDegrees();

                        while (pigeonAngle > 360) pigeonAngle -= 360;
                        while (pigeonAngle < -360) pigeonAngle += 360;

                        if (pigeonAngle < 0) pigeonAngle = 360 + pigeonAngle;

                        double angleChange = Math.abs((-180 - tagAngles[target.getFiducialId()]) + 180 + pigeonAngle);
                        if (DriverStation.getAlliance().isPresent()
                                && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red))
                            angleChange += 180;

                        while (angleChange > 180) angleChange = Math.abs(angleChange - 360);

                        if (angleChange < smallestAngleChange) {
                            smallestAngleChange = angleChange;
                            bestTarget = target;
                        }
                    }
                }
                if (bestTarget == null) {
                    return new double[]{0.0, 0.0, 0.0};
                }
                backClosestTagID = bestTarget.getFiducialId();

                double smallestDistance = PhotonUtils.calculateDistanceToTargetMeters(
                        Constants.Vision.BACK_CAMERA_POSE.getZ(), .31,
                        Constants.Vision.BACK_CAMERA_POSE.getRotation().getY(),
                        Units.degreesToRadians(bestTarget.getPitch()));

//                System.out.println("Distance: " + smallestDistance);
//                System.out.println("Yaw: " + bestTarget.getYaw());

                closestTagXBack = (Math.cos(Math.toRadians(bestTarget.getYaw() - 40)) * smallestDistance);
                closestTagYBack = -Math.sin(Math.toRadians(bestTarget.getYaw() - 40)) * smallestDistance;

//                System.out.println("Id: " + bestTarget.getFiducialId()
//                        + " X: " + closestTagXBack + " Y: " + closestTagYBack);

                closestTagXBack -= Vision.BACK_CAMERA_TO_BACK_DISTANCE;

                closestTagYawBack = bestTarget.getYaw();

                if (DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
                    closestTagXBack = -closestTagXBack;
                    closestTagYBack = -closestTagYBack;
                }
            }
        }
        return new double[]{closestTagXBack, closestTagYBack, closestTagYawBack};
    }

    public static AprilTagSubsystem getInstance() {
        if (aprilTagSubsystem == null) {
            aprilTagSubsystem = new AprilTagSubsystem();
        }
        return aprilTagSubsystem;
    }

    public void resetAutoAlignData() {
        closestTagID = -1;
        backClosestTagID = -1;

        closestTagX = 0;
        closestTagY = 0;
        closestTagYaw = 0;
        closestTagXBack = 0;
        closestTagYBack = 0;
        closestTagYawBack = 0;

        doneAutoDriving = false;
    }

    public void setDoneAutoDriving(boolean doneAutoDriving) {
        this.doneAutoDriving = doneAutoDriving;
    }

    public int getClosestTagID() {
        return closestTagID;
    }

    public int getBackClosestTagID() {
        return backClosestTagID;
    }
}