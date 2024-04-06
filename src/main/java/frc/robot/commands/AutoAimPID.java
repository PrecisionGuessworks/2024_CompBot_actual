package frc.robot.commands;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.ShotDistTable;
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
    
    private final double MaxSpeed = 5.0292; 
    private boolean inRange = false;
    private final XboxController m_joystick;
    private final Timer m_shotTimer = new Timer();
    private final ShotDistTable shotTable;
    //private ShotDistTable shotTable = new ShotDistTable();
    private double maxShotDist = 4.0; //meters
    
    final double MaxAngularRate =  1.4 * Math.PI;
    private int allianceFlip = 1;
 

    public AutoAimPID(CommandSwerveDrivetrain swerve, PhotonCamera camera, SwerveRequest.FieldCentric drive, ArmSubsystem  arm, ShooterSubsystem shooter, IntakeSubsystem intake, XboxController joystick, ShotDistTable Table) {
        m_swerve = swerve;
        m_camera = camera;
        m_drive = drive;
        m_arm = arm;
        m_shooter = shooter;
        m_intake = intake;
        m_joystick = joystick;
        shotTable = Table;
        
        //turnController.enableContinuousInput(0.0, 1.0);
    // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm, shooter, intake);

    }

    @Override
  public void initialize() {
    m_shotTimer.reset();
    inRange = false;
    m_shooter.setFeedVelocity(0);
    //m_shooter.setLaunchVelocity(Constants.Shooter.ejectVelocity);
    // Called when the command is initially scheduled.
    //m_arm.setArmAngle(Constants.Arm.PodiumlaunchAngle);
    //m_shooter.setLaunchVelocity(Constants.Shooter.PodiumlaunchVelocity);
    
        
        
        // if(currentAlliance.equals(DriverStation.Alliance.Blue)) {
        //     goalPose = new Pose2d(new Translation2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42)), new Rotation2d(Units.degreesToRadians(180)));
        // }
        // else {
        //     goalPose = new Pose2d(new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42)), new Rotation2d(0));
        // }
    
  }

  @Override
  public void execute() {
    final boolean isBPressed = m_joystick.getBButton();
    Pose2d goalPose = null;
    Translation2d tagTranslation = null;
    
    
     
    
    var alliance = DriverStation.getAlliance();
    
    
    //int speakerID = 7;

    if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            goalPose = Fiducials.AprilTags.aprilTagFiducials[6].getPose().toPose2d();
            tagTranslation = new Translation2d(goalPose.getX() , goalPose.getY() + Units.inchesToMeters(22));
            allianceFlip = 1;
        }

    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            goalPose = Fiducials.AprilTags.aprilTagFiducials[3].getPose().toPose2d();
            tagTranslation = new Translation2d(goalPose.getX() , goalPose.getY() - Units.inchesToMeters(22));
            allianceFlip = -1;
        }
    
    double filteredAngle = Constants.Arm.intakeAngle;
    double shotVelo = Constants.Shooter.ejectVelocity;
    
    
    var startRobotPose = m_swerve.getState().Pose;

    Translation2d robotPoseTranslation = startRobotPose.getTranslation();
    

    Translation2d tagToRobotVector = tagTranslation.minus(startRobotPose.getTranslation());

    var robotAngle = startRobotPose.getRotation().getRadians();

    Translation2d robotVector = new Translation2d(Math.cos(robotAngle), Math.sin(robotAngle));

    

    
    
    /* var angle1 = Math.atan2(robotVector.getY()-tagTranslation.getY(), robotVector.getX()-tagTranslation.getX());
    var angle2 = Math.atan2(robotPoseTranslation.getY()-tagTranslation.getY(), tagTranslation.getX()-robotPoseTranslation.getX()-tagTranslation.getX());

    var targetAngle = angle2 - angle1; */
     var crossVec = (robotVector.getX()*tagToRobotVector.getY()) - (robotVector.getY()*tagToRobotVector.getX());


    var magRobotVec = robotVector.getNorm();
    var magTagToRobotVector = tagToRobotVector.getNorm();

    //System.out.println("robotAngle: "+ robotAngle);


    var targetAngle = Math.asin(crossVec / (magRobotVec * magTagToRobotVector)); 

    System.out.println("targetAngle: "+Units.radiansToDegrees(targetAngle));


    var requestedAngularVelocity = (turnController.calculate(targetAngle, 0));

    //System.out.println("rotationSpeed: "+ Units.radiansToDegrees(requestedAngularVelocity));

    double goalDistance = tagToRobotVector.getNorm();

    if (goalDistance <= maxShotDist) {
      filteredAngle = Units.degreesToRadians(shotTable.calculate(goalDistance));
      System.out.println("angle calc: "+Units.radiansToDegrees(filteredAngle));
      //filteredAngle = Math.atan2(2.3, goalDistance);
      shotVelo = Constants.Shooter.PodiumlaunchVelocity;

    }

    else {
      filteredAngle = Constants.Arm.eject;
      shotVelo = Constants.Shooter.ejectVelocity;
    }

    if (goalDistance <= maxShotDist) {
      inRange = true;
    }

    else {
      inRange = false;
    }

    
  System.out.println("Distance from speaker: "+ goalDistance);
  System.out.println("Arm Angle: "+ filteredAngle);

  Supplier<SwerveRequest> regRequestSupplier =  () -> m_drive.withVelocityX(-m_joystick.getLeftY() * MaxSpeed * allianceFlip).withVelocityY(-m_joystick.getLeftX() * MaxSpeed * allianceFlip).withRotationalRate(-requestedAngularVelocity*MaxAngularRate * allianceFlip);
    m_swerve.setControl(regRequestSupplier.get());
        

  if (m_shotTimer.hasElapsed(0.3)) {
    m_arm.setArmAngle(Constants.Arm.intakeAngle);
      m_shooter.setFeedVelocity(0);
      m_shooter.setLaunchVelocity(0);

      if (m_shotTimer.hasElapsed(1.0)) {
        m_shotTimer.stop();
        //m_shotTimer.reset();
      }

  }
  else {
    m_arm.setArmAngle(filteredAngle);
    m_shooter.setLaunchVelocity(shotVelo);

    if (isBPressed) {
      m_shooter.setFeedVelocity(Constants.Shooter.scoreSpeakerFeedVelocity);
    }

  if (inRange) {
    if (withinAngleTolerance(targetAngle, 0, Constants.ShotCalc.autoAimTargetYawTol)) {
      if (m_shooter.isAtLaunchVelocity(shotVelo, Constants.Shooter.launchVelocityTolerance) && m_arm.isAtAngle(filteredAngle, Constants.Arm.launchAngleTolerance)) {
        // m_armSubsystem.resetEncoders(Constants.Arm.launchAngle);
        m_shooter.setFeedVelocity(Constants.Shooter.scoreSpeakerFeedVelocity);
        m_shotTimer.start();
        
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