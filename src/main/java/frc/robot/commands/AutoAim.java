package frc.robot.commands;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.vision.Fiducials;


public class AutoAim extends Command{
    private final CommandSwerveDrivetrain m_swerve;
    private final PhotonCamera m_camera;
    private final int CAMERA_HEIGHT_METERS = 0;
    private final ArmSubsystem m_armSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;

    public AutoAim(CommandSwerveDrivetrain swerve, PhotonCamera camera, ArmSubsystem arm, ShooterSubsystem shooter) {
        m_swerve = swerve;
        m_camera = camera;
        m_armSubsystem = arm;
        m_shooterSubsystem = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerve, arm, shooter);

    }

    @Override
  public void initialize() {
    // Called when the command is initially scheduled.
    m_shooterSubsystem.setLaunchVelocity(Constants.Shooter.launchVelocity);
  }

  @Override
  public void execute() {
    // Called every time Command is scheduled
    var result = m_camera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    var alliance = DriverStation.getAlliance();

    Pose3d tag_pose = new Pose3d();
    double filteredAngle = Constants.Arm.intakeAngle;
    

    if (hasTargets) {
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            tag_pose = Fiducials.AprilTags.BlueSpeakerTag.pose;
        }

        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            tag_pose = Fiducials.AprilTags.RedSpeakerTag.pose;
        }
        
        Pose2d robotPose = m_swerve.getState().Pose;


        Rotation2d targetYaw = PhotonUtils.getYawToPose(robotPose, tag_pose.toPose2d());

        Pose2d targetPose = new Pose2d(robotPose.getX(), robotPose.getY(), targetYaw);
        m_swerve.followTrajectoryCommand(targetPose);

        Translation2d tagPose2d = new Translation2d(tag_pose.getX(), tag_pose.getY());
        Translation2d tagVector = tagPose2d.minus(robotPose.getTranslation());
        final double goalDistance = tagVector.getNorm();

        double armTheta = Math.atan2((tag_pose.getZ()-CAMERA_HEIGHT_METERS), goalDistance);

        filteredAngle = Math.max(Math.min(armTheta, Constants.Arm.maxAngle), Constants.Arm.intakeAngle);
        
    }
    m_armSubsystem.setArmAngle(filteredAngle);

    if ( m_shooterSubsystem.isAtLaunchVelocity(Constants.Shooter.launchVelocity, Constants.Shooter.launchVelocityTolerance) && m_armSubsystem.isAtAngle(filteredAngle, Constants.Arm.launchAngleTolerance)) {
       // m_armSubsystem.resetEncoders(Constants.Arm.launchAngle);
        
        m_shooterSubsystem.setFeedVelocity(Constants.Shooter.scoreSpeakerFeedVelocity);
    }
    

  }

  @Override
  public void end(boolean interrupted) {
    //Called when command ends or is interrupted
    m_shooterSubsystem.setFeedVelocity(0);
    m_shooterSubsystem.setLaunchVelocity(0);
  }

  @Override
  public boolean isFinished() {
    //Called when Command is finished
    return false;
  }
    


    
}
