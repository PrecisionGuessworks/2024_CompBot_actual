package frc.robot.subsystems;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.motorcontrol.TalonFx;
import frc.robot.motorcontrol.configurations.TalonFxConfiguration;

public class PresPoseEstimator  extends SubsystemBase{
    private final PhotonCamera m_photonCamera;
    private final CommandSwerveDrivetrain m_swerveDrivetrain;
    private final AprilTagFieldLayout m_aprilTagFieldLayout;


    private final Field2d m_field2d = new Field2d();

    private double previousPipelineTimestamp = 0;

    private final Transform3d robotToCam;;

    private final PhotonPoseEstimator m_photonPoseEstimator;

    
    public PresPoseEstimator(PhotonCamera photonCamera, CommandSwerveDrivetrain swerveDrivetrain, Transform3d RToCam) {
      //Body
      m_photonCamera = photonCamera;
      m_swerveDrivetrain = swerveDrivetrain;
      m_aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
      robotToCam = RToCam;
    
     m_photonPoseEstimator = new PhotonPoseEstimator(m_aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_photonCamera, robotToCam);

   
      //Show scheduler status in SmartDashboard.
      SmartDashboard.putData(this);

    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        m_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return m_photonPoseEstimator.update();
    }

    public Pose2d getCurrentPose() {
        return m_swerveDrivetrain.getState().Pose;
      }
   
    @Override
    public void periodic() {
        var pipelineResult = m_photonCamera.getLatestResult();
        

        if (pipelineResult.hasTargets()) {
          var resultTimestamp = pipelineResult.getTimestampSeconds();
          m_swerveDrivetrain.addVisionMeasurement(getEstimatedGlobalPose(getCurrentPose()).get().estimatedPose.toPose2d(), resultTimestamp);
  
        }

        
      // This method will be called once per scheduler run
    }
  
    // --- BEGIN STUFF FOR SIMULATION ---
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
    // --- END STUFF FOR SIMULATION ---
  }
    

