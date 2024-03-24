package frc.robot.commands;
import java.util.List;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.vision.Fiducials;


public class AutoAimVision extends Command{
    private final CommandSwerveDrivetrain m_swerve;
    private final ArmSubsystem m_arm;
    private final ShooterSubsystem m_shooter;
    private final SwerveRequest.FieldCentric m_drive;
    private final PhotonCamera m_camera;
    private final PIDController turnController = new PIDController(0.5, 0, 0.4);
    private final IntakeSubsystem m_intake;
    int shottimeout = 0;
    
    private final double MaxSpeed = 5.0292; 
    private boolean inRange = false;
    private final XboxController m_joystick;
    
    final double MaxAngularRate = 0.4 * Math.PI;

    public AutoAimVision(CommandSwerveDrivetrain swerve, PhotonCamera camera, SwerveRequest.FieldCentric drive, ArmSubsystem  arm, ShooterSubsystem shooter, IntakeSubsystem intake, XboxController joystick) {
        m_swerve = swerve;
        m_camera = camera;
        m_drive = drive;
        m_arm = arm;
        m_shooter = shooter;
        m_intake = intake;
        m_joystick = joystick;
      //turnController.enableContinuousInput(0.0, 1.0);
    // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm, shooter, intake);

    }

    @Override
  public void initialize() {
    // Called when the command is initially scheduled.
    //m_arm.setArmAngle(Constants.Arm.PodiumlaunchAngle);
    //m_shooter.setLaunchVelocity(Constants.Shooter.PodiumlaunchVelocity);
    
  }

  @Override
  public void execute() {
    
     //m_arm.setArmAngle(Constants.Arm.PodiumlaunchAngle);
     var result = m_camera.getLatestResult();
    var alliance = DriverStation.getAlliance();
    
    int speakerID = 7;

    if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            speakerID = Fiducials.AprilTags.aprilTagFiducials[6].getID();
        }

    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            speakerID = Fiducials.AprilTags.aprilTagFiducials[3].getID();
        }
    
    

    //m_shooter.setLaunchVelocity(Constants.Shooter.PodiumlaunchVelocity);
    //m_arm.setArmAngle(Constants.Arm.PodiumlaunchAngle);
        
      
    Supplier<SwerveRequest> regRequestSupplier =  () -> m_drive.withVelocityX(-m_joystick.getLeftY() * MaxSpeed).withVelocityY(-m_joystick.getLeftX() * MaxSpeed).withRotationalRate(-m_joystick.getRightX()*MaxAngularRate);
    m_swerve.setControl(regRequestSupplier.get());
 
    if (result.hasTargets()) {
      var targetList = result.getTargets();
      for (int i = 0; i < targetList.size(); i++) {
        var target = targetList.get(i);
        System.out.println("target id"+ target.getFiducialId());

        if (target.getFiducialId() == speakerID) {
          System.out.println("tag yaw: "+ target.getYaw());
          double rotationSpeed = turnController.calculate(target.getYaw(), 0);
          System.out.println("rotationSpeed: "+ rotationSpeed);

          Supplier<SwerveRequest> requestSupplier =  () -> m_drive.withVelocityX(-m_joystick.getLeftY() * MaxSpeed).withVelocityY(-m_joystick.getLeftX() * MaxSpeed).withRotationalRate(rotationSpeed*MaxAngularRate);
          m_swerve.setControl(requestSupplier.get());
          
//prolly wont use
          
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