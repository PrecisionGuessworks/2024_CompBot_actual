package frc.robot.subsystems;

//import static frc.robot.Constants.VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
//import frc.robot.util.FieldConstants;


/**
 * Runnable that gets AprilTag data from PhotonVision.
 */
public class PresPoseEstimatorMulti implements Runnable {

  private final PhotonPoseEstimator photonPoseEstimator;
  private final PhotonCamera photonCamera;
  public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
  public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(651.25);
  public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(315.5);
  private final CommandSwerveDrivetrain m_swerveDrivetrain;
  //private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();

  public PresPoseEstimatorMulti(PhotonCamera cameraName, Transform3d robotToCamera, CommandSwerveDrivetrain swerveDrivetrain) {
    this.photonCamera = cameraName;
    m_swerveDrivetrain = swerveDrivetrain;

    PhotonPoseEstimator photonPoseEstimator = null;
    try {
      var layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
      // PV estimates will always be blue, they'll get flipped by robot thread
      layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
      if (photonCamera != null) {
        photonPoseEstimator = new PhotonPoseEstimator(
            layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCamera, robotToCamera);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
      }
    } catch (Exception e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      photonPoseEstimator = null;
    }
    this.photonPoseEstimator = photonPoseEstimator;
  }

  @Override
  public void run() {
    System.out.println("Robot Pose: "+m_swerveDrivetrain.getState().Pose);
    // Get AprilTag data
    if (photonPoseEstimator != null && photonCamera != null) {
      var photonResults = photonCamera.getLatestResult();

      if (photonResults.hasTargets()
          && (photonResults.targets.size() > 1
              || photonResults.targets.get(0).getPoseAmbiguity() < APRILTAG_AMBIGUITY_THRESHOLD)) {

                var imageCaptureTime = photonResults.getTimestampSeconds();

        photonPoseEstimator.update(photonResults).ifPresent(estimatedRobotPose -> {
          var estimatedPose = estimatedRobotPose.estimatedPose;
          // Make sure the measurement is on the field

          if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= FIELD_LENGTH_METERS
              && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= FIELD_WIDTH_METERS) {
            //atomicEstimatedRobotPose.set(estimatedRobotPose);

             m_swerveDrivetrain.addVisionMeasurement(
                    estimatedPose.toPose2d(), imageCaptureTime);
          }
        });
      }
    }
  }



}