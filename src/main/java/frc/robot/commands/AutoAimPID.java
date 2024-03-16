package frc.robot.commands;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.vision.Fiducials;


public class AutoAimPID extends Command{
    private final CommandSwerveDrivetrain m_swerve;
    private final ArmSubsystem m_arm;
    private final ShooterSubsystem m_shooter;
    private final SwerveRequest.FieldCentric m_drive;
    private final PhotonCamera m_camera;
    private final PIDController turnController = new PIDController(1.0, 0, 0.1);
    private final IntakeSubsystem m_intake;
    int shottimeout = 0;
    
    final double MaxAngularRate = 2 * Math.PI;

    public AutoAimPID(CommandSwerveDrivetrain swerve, PhotonCamera camera, SwerveRequest.FieldCentric drive, ArmSubsystem  arm, ShooterSubsystem shooter, IntakeSubsystem intake) {
        m_swerve = swerve;
        m_camera = camera;
        m_drive = drive;
        m_arm = arm;
        m_shooter = shooter;
        m_intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerve, arm, shooter, intake);

    }

    @Override
  public void initialize() {
    // Called when the command is initially scheduled.
    m_arm.setArmAngle(Constants.Arm.PodiumlaunchAngle);
    m_shooter.setLaunchVelocity(Constants.Shooter.PodiumlaunchVelocity);
    
  }

  @Override
  public void execute() {
    

    if (!m_intake.isBeamBreakTriggered()) {
      
      shottimeout++;
    
    
    
    if (shottimeout >= Constants.Arm.ShootTimeout){
       
        m_shooter.setFeedVelocity(0);
        m_shooter.setLaunchVelocity(0);
        m_arm.setArmAngle(Constants.Arm.intakeAngle);
        }
        
  } 
  
  else{
    
     m_arm.setArmAngle(Constants.Arm.PodiumlaunchAngle);
     var result = m_camera.getLatestResult();
    var alliance = DriverStation.getAlliance();
    
    int speakerID = 7;

    if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            speakerID = Fiducials.AprilTags.aprilTagFiducials[6].getID();
        }

    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            speakerID = Fiducials.AprilTags.aprilTagFiducials[3].getID();
        }
    
    if (result.hasTargets()) {
      var targetList = result.getTargets();
      for (int i = 0; i < targetList.size(); i++) {
        var target = targetList.get(i);

        if (target.getFiducialId() == speakerID) {
          double rotationSpeed = -turnController.calculate(target.getYaw(), 0);
          m_swerve.applyRequest(() -> m_drive.withRotationalRate(rotationSpeed*MaxAngularRate));

          if (withinAngleTolerance(target.getYaw(), Constants.ShotCalc.autoAimTargetYaw, Constants.ShotCalc.autoAimTargetYawTol)) {

            if ( m_shooter.isAtLaunchVelocity(Constants.Shooter.PodiumlaunchVelocity, Constants.Shooter.PodiumlaunchVelocityTolerance) && m_arm.isAtAngle(Constants.Arm.PodiumlaunchAngle, Constants.Arm.PodiumlaunchAngleTolerance)) {
              // m_armSubsystem.resetEncoders(Constants.Arm.launchAngle);
              shottimeout = 0;
                m_shooter.setFeedVelocity(Constants.Shooter.scoreSpeakerFeedVelocity);
                   
            }   
        }
      }
    }
    }        
        
    }
    
    // Called every time Command is scheduled
       
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.setFeedVelocity(0);
    m_shooter.setLaunchVelocity(0);
    //Called when command ends or is interrupted
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
