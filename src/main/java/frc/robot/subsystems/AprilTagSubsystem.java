//  package frc.robot.subsystems;
//
//  import java.io.IOException;
//  import java.util.ArrayList;
//  import java.util.List;
//  import java.util.Optional;
//
//  import edu.wpi.first.apriltag.AprilTagFieldLayout;
//  import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.apriltag.AprilTagPoseEstimate;
// import edu.wpi.first.wpilibj.Filesystem;
//  import frc.robot.constants.TunerConstants;
//  import org.photonvision.EstimatedRobotPose;
//  import org.photonvision.PhotonCamera;
//  import org.photonvision.PhotonPoseEstimator;
//  import org.photonvision.PhotonPoseEstimator.PoseStrategy;
//  import org.photonvision.proto.Photon;
//  import org.photonvision.targeting.PhotonPipelineResult;
//  import org.photonvision.targeting.PhotonTrackedTarget;
//
//  import edu.wpi.first.math.Matrix;
//  import edu.wpi.first.math.Pair;
//  import edu.wpi.first.math.VecBuilder;
//  import edu.wpi.first.math.geometry.Pose2d;
//  import edu.wpi.first.math.numbers.N1;
//  import edu.wpi.first.math.numbers.N3;
//  import edu.wpi.first.wpilibj2.command.SubsystemBase;
//  import frc.robot.constants.Constants;
//  import org.photonvision.simulation.VisionTargetSim;
//  import edu.wpi.first.math.geometry.Pose3d;
//
//  public class AprilTagSubsystem extends SubsystemBase {
//
//      private final PhotonCamera camera;
//      //    private final PhotonCamera camera_Back;
//      private final PhotonPoseEstimator photonEstimator;
//      //    private final PhotonPoseEstimator photonEstimatorBack;
//      private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
//      private double lastEstTimestamp = 0;
//      private double lastEstTimestampBack = 0;
//      private PhotonPipelineResult lastResult;
//      //    private PhotonPipelineResult lastResultBack;
//      private AprilTagFieldLayout customLayout;
//      private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
//  //    private AprilTagFieldLayout backCameraLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
//      private AprilTagPoseEstimate poseEstimate;
//
// //      public AprilTagSubsystem() {
// //          camera = new PhotonCamera(Constants.Vision.kCameraName);
// //  //        camera_Back = new PhotonCamera(Constants.Vision.kCameraName_Back);
// //          try {
// //              customLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/configs/2024-crescendo.json");
// //          } catch (IOException e) {
// //              throw new RuntimeException(e);
// //          }
//
//
//         photonEstimator =
//                  new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.Vision.kRobotToCam);
//  //        photonEstimatorBack =
//  //                new PhotonPoseEstimator(backCameraLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera_Back, Constants.Vision.kRobotToCamBack);
//          photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
//  //        photonEstimatorBack.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
//
//          lastResult = camera.getLatestResult();
//  //        lastResultBack = camera_Back.getLatestResult();
//      }
//
//      @Override
//      public void periodic() {
//      }
//
//
//
//
//
//      public void getLatestResult() {
//      if (camera.getLatestResult().hasTargets()) {
//              lastResult = camera.getLatestResult();
//          }
//      }
//
//      public void getLatestResultBack() {
//  //        if (camera_Back.getLatestResult().hasTargets()) {
//  //            lastResultBack = camera_Back.getLatestResult();
//  //        }
//      }
//
//      // public int[] getTargets() {
//      //     List<PhotonTrackedTarget> targets = camera.getLatestResult().targets;
//      //     ArrayList targetids = new ArrayList<>() {
//      //         {
//      //             add(0);
//      //             add(0);
//      //             add(0);
//      //         }
//      //     }
//      //     for (PhotonTrackedTarget target : targets) {
//      //         taretids
//      //     }
//      // }
//
//      public Pair<Optional<EstimatedRobotPose>, Double> getEstimatedGlobalPose() {
//          photonEstimator.update(lastResult);
//
//          getLatestResult();
//
//          // filtering stages
//          // Ensure the result is
//          if (lastResult.getTimestampSeconds() <= lastEstTimestamp) {
//              return new Pair<>(Optional.empty(), 0d);
//          } else if (lastResult.getTargets().size() < 2) {
//              return new Pair<>(Optional.empty(), 0d);
//          } else {
//              return new Pair<>(photonEstimator.update(lastResult), lastResult.getTimestampSeconds());
//          }
//
//      }
//
//      public Pair<Optional<EstimatedRobotPose>, Double> getEstimatedGlobalPoseBack() {
//  //        photonEstimatorBack.setReferencePose(drivetrain.getPose());
//  //
//  //        getLatestResultBack();
//  //
//  //        // filtering stages
//  //        // Ensure the result is
//  //        if (lastResultBack.getTimestampSeconds() <= lastEstTimestampBack) {
//  //            return new Pair<>(Optional.empty(), 0d);
//  //        } else if (lastResultBack.getTargets().size() < 2) {
//  //            return new Pair<>(Optional.empty(), 0d);
//  //        } else {
//  //            return new Pair<>(photonEstimatorBack.update(lastResultBack), lastResultBack.getTimestampSeconds());
//  //        }
//          return new Pair<>(Optional.empty(), 0d);
//      }
//
//      public double getAmbiguity() {
//         //  return lastResult.getMultiTagResult().estimatedPose.ambiguity;
//
//         return poseEstimate.getAmbiguity();
//      }
//
//      public double getAmbiguityBack() {
//  //        return lastResultBack.getMultiTagResult().estimatedPose.ambiguity;
//          return 0;
//      }
//
//
//      public double getTimestamp() {
//          return lastResult.getTimestampSeconds();
//      }
//
// //      public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
// //          var estStdDevs = Constants.Vision.kSingleTagStdDevs;
// //          var targets = lastResult.getTargets();
// //          int numTags = 0;
// //          double avgDist = 0;
// //          for (var tgt : targets) {
// //              var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
// //              if (tagPose.isEmpty()) continue;
// //              numTags++;
// //              avgDist +=
// //                      tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
// //          }
// //          if (numTags == 0) return estStdDevs;
// //          avgDist /= numTags;
// //          // Decrease std devs if multiple targets are visible
// //          if (numTags > 1) estStdDevs = Constants.Vision.kMultiTagStdDevsTeleop;
// //          // Increase std devs based on (average) distance
// //          if (numTags == 1 && avgDist > 4)
// //              estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
// //          else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
//
// //          return estStdDevs;
// //      }
// //  }
