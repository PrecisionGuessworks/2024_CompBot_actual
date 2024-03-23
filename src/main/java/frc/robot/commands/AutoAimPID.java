package frc.robot.commands;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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


public class AutoAimPID extends Command{
    private final CommandSwerveDrivetrain m_swerve;
    private final ArmSubsystem m_arm;
    private final ShooterSubsystem m_shooter;
    private final SwerveRequest.FieldCentric m_drive;
    private final PhotonCamera m_camera;
    private final PIDController turnController = new PIDController(1.0, 0, 0.0);
    private final IntakeSubsystem m_intake;
    int shottimeout = 0;
    
    private final double MaxSpeed = 5.0292; 
    private boolean inRange = false;
    private final XboxController m_joystick;
    
    final double MaxAngularRate =  2 * Math.PI;
    private Rotation2d deltaTheta;
    private Rotation2d targetAngle;

    public AutoAimPID(CommandSwerveDrivetrain swerve, PhotonCamera camera, SwerveRequest.FieldCentric drive, ArmSubsystem  arm, ShooterSubsystem shooter, IntakeSubsystem intake, XboxController joystick) {
        m_swerve = swerve;
        m_camera = camera;
        m_drive = drive;
        m_arm = arm;
        m_shooter = shooter;
        m_intake = intake;
        m_joystick = joystick;
        turnController.enableContinuousInput(0.0, 1.0);
    // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm, shooter, intake);

    }

    @Override
  public void initialize() {
    // Called when the command is initially scheduled.
    Pose2d goalPose = null;
    
  final var robotPose = m_swerve.getState().Pose;
    
    var alliance = DriverStation.getAlliance();
    
    
    //int speakerID = 7;

    if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            goalPose = Fiducials.AprilTags.aprilTagFiducials[6].getPose().toPose2d();
        }

    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            goalPose = Fiducials.AprilTags.aprilTagFiducials[3].getPose().toPose2d();
        }

        final var startingAngle = robotPose.getRotation();
        final var endAngle = goalPose.getTranslation().minus(robotPose.getTranslation()).getAngle().plus(Rotation2d.fromDegrees(180.0));
        deltaTheta = endAngle.minus(startingAngle);

        targetAngle = Rotation2d.fromDegrees(m_swerve.getRotation3d().toRotation2d().getDegrees() + deltaTheta.getDegrees());
    
  }

  

  @Override
  public void execute() {

     var currentAngle = m_swerve.getState().Pose.getRotation();

       var requestedAngularVelocity = Rotation2d.fromDegrees(MathUtil.clamp(
            turnController.calculate(currentAngle.getDegrees(), targetAngle.getDegrees()),
            -Units.radiansToDegrees(MaxAngularRate),
            Units.radiansToDegrees(MaxAngularRate)
        ));
         
    Supplier<SwerveRequest> regRequestSupplier =  () -> m_drive.withVelocityX(-m_joystick.getLeftY() * MaxSpeed).withVelocityY(-m_joystick.getLeftX() * MaxSpeed).withRotationalRate(requestedAngularVelocity.getRadians());
    m_swerve.setControl(regRequestSupplier.get());

        
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