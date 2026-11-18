package frc.robot.subsystems.swervedrive;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;

/**
 * PhotonVision integration class to aid in accurate swerve odometry.
 */
public class Vision {

  /**
   * AprilTag Field Layout for the season.
   */
  public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(
      AprilTagFields.k2023ChargedUp); // Replace with a valid field layout

  /**
   * Ambiguity threshold used to filter out unstable poses.
   */

  /**
   * PhotonVision Simulation System.
   */
  public VisionSystemSim visionSim;

  /**
   * Current pose supplier referencing SwerveDrive.
   */
  private Supplier<Pose2d> currentPose;

  /**
   * Field telemetry object from SwerveDrive.
   */
  private Field2d field2d;

  /**
   * Constructor for the Vision class.
   *
   * @param currentPose Current pose supplier, referencing {@link SwerveDrive#getPose()}
   * @param field       Current field object from {@link SwerveDrive#field}
   */
  public Vision(Supplier<Pose2d> currentPose, Field2d field) {
    this.currentPose = currentPose;
    this.field2d = field;

    if (Robot.isSimulation()) {
      visionSim = new VisionSystemSim("Vision");
      visionSim.addAprilTags(fieldLayout);

      for (Cameras c : Cameras.values()) {
        c.addToVisionSim(visionSim);
      }
    }
  }

  /**
   * Calculates a target pose relative to an AprilTag on the field.
   */
  public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset) {
    Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);
    if (aprilTagPose3d.isPresent()) {
      return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
    } else {
      throw new RuntimeException("Cannot get AprilTag " + aprilTag + " from field layout.");
    }
  }

  /**
   * Update pose estimation inside SwerveDrive using vision measurements.
   */
  public void updatePoseEstimation(SwerveDrive swerveDrive) {
    if (SwerveDriveTelemetry.isSimulation && swerveDrive.getSimulationDriveTrainPose().isPresent()) {
      visionSim.update(swerveDrive.getSimulationDriveTrainPose().get());
    }
    
    for (Cameras camera : Cameras.values()) {
      Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);
      if (poseEst.isPresent()) {
        var pose = poseEst.get();
        swerveDrive.addVisionMeasurement(
            pose.estimatedPose.toPose2d(),
            pose.timestampSeconds,
            camera.curStdDevs);
      }
    }
  }

  /**
   * Generates the estimated robot pose for a specific camera.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Cameras camera) {
    Optional<EstimatedRobotPose> poseEst = camera.getEstimatedGlobalPose();
    if (Robot.isSimulation()) {
      Field2d debugField = visionSim.getDebugField();
      poseEst.ifPresentOrElse(
          est -> debugField.getObject("VisionEstimation").setPose(est.estimatedPose.toPose2d()),
          () -> debugField.getObject("VisionEstimation").setPoses());
    }
    return poseEst;
  }

  public double getDistanceFromAprilTag(int id) {
    Optional<Pose3d> tag = fieldLayout.getTagPose(id);
    return tag.map(pose3d -> PhotonUtils.getDistanceToPose(currentPose.get(), pose3d.toPose2d())).orElse(-1.0);
  }

  public PhotonTrackedTarget getTargetFromId(int id, Cameras camera) {
    for (PhotonPipelineResult result : camera.resultsList) {
      if (result.hasTargets()) {
        for (PhotonTrackedTarget i : result.getTargets()) {
          if (i.getFiducialId() == id) {
            return i;
          }
        }
      }
    }
    return null;
  }

  public VisionSystemSim getVisionSim() {
    return visionSim;
  }

  public void updateVisionField() {
    List<PhotonTrackedTarget> targets = new ArrayList<>();
    for (Cameras c : Cameras.values()) {
      if (!c.resultsList.isEmpty()) {
        PhotonPipelineResult latest = c.resultsList.get(0);
        if (latest.hasTargets()) {
          targets.addAll(latest.targets);
        }
      }
    }

    List<Pose2d> poses = new ArrayList<>();
    for (PhotonTrackedTarget target : targets) {
      if (fieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
        poses.add(fieldLayout.getTagPose(target.getFiducialId()).get().toPose2d());
      }
    }

    field2d.getObject("tracked targets").setPoses(poses);
  }

  /**
   * Camera Enum to configure physical and simulated camera setups.
   */
  enum Cameras {
    LEFT_CAM(
        "left",
        new Rotation3d(0, Math.toRadians(-24.094), Math.toRadians(30)),
        new Translation3d(Units.inchesToMeters(12.056), Units.inchesToMeters(10.981), Units.inchesToMeters(8.44)),
        VecBuilder.fill(4, 4, 8),
        VecBuilder.fill(0.5, 0.5, 1)),

    RIGHT_CAM(
        "right",
        new Rotation3d(0, Math.toRadians(-24.094), Math.toRadians(-30)),
        new Translation3d(Units.inchesToMeters(12.056), Units.inchesToMeters(-10.981), Units.inchesToMeters(8.44)),
        VecBuilder.fill(4, 4, 8),
        VecBuilder.fill(0.5, 0.5, 1)),

    CENTER_CAM(
        "center",
        new Rotation3d(0, Units.degreesToRadians(18), 0),
        new Translation3d(Units.inchesToMeters(-4.628), Units.inchesToMeters(-10.687), Units.inchesToMeters(16.129)),
        VecBuilder.fill(4, 4, 8),
        VecBuilder.fill(0.5, 0.5, 1));

    public final Alert latencyAlert;
    public final PhotonCamera camera;
    public final PhotonPoseEstimator poseEstimator;
    private final Matrix<N3, N1> singleTagStdDevs;
    private final Matrix<N3, N1> multiTagStdDevs;
    private final Transform3d robotToCamTransform;
    public Matrix<N3, N1> curStdDevs;
    public Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();
    public PhotonCameraSim cameraSim;
    public List<PhotonPipelineResult> resultsList = new ArrayList<>();

    Cameras(
        String name,
        Rotation3d robotToCamRotation,
        Translation3d robotToCamTranslation,
        Matrix<N3, N1> singleTagStdDevs,
        Matrix<N3, N1> multiTagStdDevsMatrix) {
      latencyAlert = new Alert("'" + name + "' Camera is experiencing high latency.", AlertType.kWarning);
      camera = new PhotonCamera(name);
      robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);

      poseEstimator = new PhotonPoseEstimator(
          Vision.fieldLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          robotToCamTransform);
      poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      this.singleTagStdDevs = singleTagStdDevs;
      this.multiTagStdDevs = multiTagStdDevsMatrix;

      if (Robot.isSimulation()) {
        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(100));
        cameraProp.setCalibError(0.25, 0.08);
        cameraProp.setFPS(30);
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        cameraSim = new PhotonCameraSim(camera, cameraProp);
        cameraSim.enableDrawWireframe(true);
      }
    }

    public void addToVisionSim(VisionSystemSim systemSim) {
      if (Robot.isSimulation()) {
        systemSim.addCamera(cameraSim, robotToCamTransform);
      }
    }

    /**
     * Get the result with the least ambiguity from the best tracked target within the cache.
     */
    public Optional<PhotonPipelineResult> getBestResult() {
      if (resultsList.isEmpty()) {
        return Optional.empty();
      }

      PhotonPipelineResult bestResult = resultsList.get(0);
      if (bestResult.getTargets().isEmpty()) {
        return Optional.of(bestResult);
      }

      double ambiguity = bestResult.getBestTarget().getPoseAmbiguity();
      for (PhotonPipelineResult result : resultsList) {
        if (!result.getTargets().isEmpty()) {
          double currentAmbiguity = result.getBestTarget().getPoseAmbiguity();
          if (currentAmbiguity < ambiguity && currentAmbiguity > 0) {
            bestResult = result;
            ambiguity = currentAmbiguity;
          }
        }
      }
      return Optional.of(bestResult);
    }

    /**
     * Get the latest result from the current cache.
     */
    public Optional<PhotonPipelineResult> getLatestResult() {
      return resultsList.isEmpty() ? Optional.empty() : Optional.of(resultsList.get(0));
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
      updateUnreadResults();
      return estimatedRobotPose;
    }

    private void updateUnreadResults() {
      resultsList = Robot.isReal() ? camera.getAllUnreadResults() : cameraSim.getCamera().getAllUnreadResults();
      resultsList.sort((a, b) -> Double.compare(a.getTimestampSeconds(), b.getTimestampSeconds()));
      if (!resultsList.isEmpty()) {
        updateEstimatedGlobalPose();
      }
    }

    private void updateEstimatedGlobalPose() {
      Optional<EstimatedRobotPose> visionEst = Optional.empty();
      for (var change : resultsList) {
        visionEst = poseEstimator.update(change);
        updateEstimationStdDevs(visionEst, change.getTargets());
      }
      estimatedRobotPose = visionEst;
    }

    private void updateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
      if (estimatedPose.isEmpty()) {
        curStdDevs = singleTagStdDevs;
      } else {
        var estStdDevs = singleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;

        for (var tgt : targets) {
          var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
          if (tagPose.isEmpty()) {
            continue;
          }
          numTags++;
          avgDist += tagPose.get().toPose2d().getTranslation()
              .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0) {
          curStdDevs = singleTagStdDevs;
        } else {
          avgDist /= numTags;
          if (numTags > 1) {
            estStdDevs = multiTagStdDevs;
          }
          if (numTags == 1 && avgDist > 4) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
          } else {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
          }
          curStdDevs = estStdDevs;
        }
      }
    }
  }
}
