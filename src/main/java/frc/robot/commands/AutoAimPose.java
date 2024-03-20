package frc.robot.commands;
import java.util.List;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.ShotDistTable;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.vision.Fiducials;


public class AutoAimPose extends Command{
    private final CommandSwerveDrivetrain m_swerve;
    private final PhotonCamera m_camera;
    private final ArmSubsystem m_arm;
    private final ShooterSubsystem m_shooter;
    private final SwerveRequest.FieldCentric m_drive;
    private final double MaxAngularRate = 2 * Math.PI;
    private final double MaxSpeed = 5.0292; 
    private boolean inRange = false;
    private final XboxController m_joystick;
    private final Timer m_shotTimer = new Timer();
    
    private final PIDController turnController = new PIDController(1.0, 0, 0.1);

    public AutoAimPose(CommandSwerveDrivetrain swerve, PhotonCamera camera, ArmSubsystem arm, ShooterSubsystem shooter, Transform3d robotToCam, SwerveRequest.FieldCentric drive, XboxController joystick) {
        m_swerve = swerve;
        m_camera = camera;
        m_arm = arm;
        m_shooter = shooter;
        m_drive = drive;
        m_joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerve, arm, shooter);

    }

    @Override
  public void initialize() {
    // Called when the command is initially scheduled.
    m_shooter.setFeedVelocity(0);
    m_shooter.setLaunchVelocity(Constants.Shooter.ejectVelocity);
    m_shotTimer.reset();
  }

  @Override
  public void execute() {
    // Called every time Command is scheduled
    var alliance = DriverStation.getAlliance();
    Pose3d tag_pose = new Pose3d();

    int speakerID = 7;


    if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            tag_pose = Fiducials.AprilTags.aprilTagFiducials[6].getPose();
            speakerID = Fiducials.AprilTags.aprilTagFiducials[6].getID();
        }

        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            tag_pose = Fiducials.AprilTags.aprilTagFiducials[3].getPose();
            speakerID = Fiducials.AprilTags.aprilTagFiducials[3].getID();
        }

    
    double filteredAngle = Constants.Arm.intakeAngle;
    double shotVelo = Constants.Shooter.ejectVelocity;
    Pose2d robotPose = m_swerve.getState().Pose;
    

    Translation2d tagPose2d = new Translation2d(tag_pose.getX(), tag_pose.getY());
    Translation2d tagVector = tagPose2d.minus(robotPose.getTranslation());
    double goalDistance = tagVector.getNorm();

    if (goalDistance < ShotDistTable.maxArmDist) {
      filteredAngle = Math.atan2(Constants.ShotCalc.speakerHeight, goalDistance);
      shotVelo = Constants.Shooter.PodiumlaunchVelocity;

    }

    else {
      filteredAngle = Constants.Arm.eject;
      shotVelo = Constants.Shooter.ejectVelocity;
    }

    if (goalDistance < ShotDistTable.maxShotDist) {
      inRange = true;
    }

    else {
      inRange = false;
    }

    var result = m_camera.getLatestResult();

    m_shooter.setLaunchVelocity(shotVelo);
    m_arm.setArmAngle(filteredAngle);
    
 
    if (result.hasTargets()) {
      var targetList = result.getTargets();
      for (int i = 0; i < targetList.size(); i++) {
        var target = targetList.get(i);
        System.out.println("target id"+ target.getFiducialId());

        if (target.getFiducialId() == speakerID) {
          System.out.println("tag yaw: "+ target.getYaw());
          double rotationSpeed = -turnController.calculate(target.getYaw(), 0);

          Supplier<SwerveRequest> requestSupplier =  () -> m_drive.withVelocityX(-m_joystick.getLeftY() * MaxSpeed).withVelocityY(-m_joystick.getLeftX() * MaxSpeed).withRotationalRate(rotationSpeed*MaxAngularRate);
          m_swerve.setControl(requestSupplier.get());
          

          if (withinAngleTolerance(target.getYaw(), Constants.ShotCalc.autoAimTargetYaw, Constants.ShotCalc.autoAimTargetYawTol)) {

            if ( m_shooter.isAtLaunchVelocity(shotVelo, Constants.Shooter.PodiumlaunchVelocityTolerance) 
            && m_arm.isAtAngle(filteredAngle, Constants.Arm.PodiumlaunchAngleTolerance)
            && inRange) {
              // m_armSubsystem.resetEncoders(Constants.Arm.launchAngle);
              
                m_shooter.setFeedVelocity(Constants.Shooter.scoreSpeakerFeedVelocity);
                m_shotTimer.start();
                   
            }   
        }
      }
      else {

       Supplier<SwerveRequest> requestSupplier =  () -> m_drive.withVelocityX(-m_joystick.getLeftY() * MaxSpeed).withVelocityY(-m_joystick.getLeftX() * MaxSpeed).withRotationalRate(-m_joystick.getRightX()*MaxAngularRate);
          m_swerve.setControl(requestSupplier.get());
        
      }
    }
    } 
    
    if (m_shotTimer.hasElapsed(0.3)) {
      m_shooter.setFeedVelocity(0);
      m_shooter.setLaunchVelocity(0);
      m_arm.setArmAngle(Constants.Arm.intakeAngle);
      m_shotTimer.stop();


    }
  }
  

  @Override
  public void end(boolean interrupted) {
    //Called when command ends or is interrupted
    m_shooter.setFeedVelocity(0);
    m_shooter.setLaunchVelocity(0);
  }

  @Override
  public boolean isFinished() {
    //Called when Command is finished
    return false;
  }

  public boolean withinAngleTolerance(double yaw, double targetYaw, double targetYawTol) {
    return (Math.abs(targetYaw - yaw) <= targetYawTol);
  }
    


    
}
