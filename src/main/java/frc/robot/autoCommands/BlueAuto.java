package frc.robot.autoCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
impoimport edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class BlueAuto extends Command{
    private final CommandSwerveDrivetrain m_swerve;
    private final ArmSubsystem m_armSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;

    public BlueAuto(CommandSwerveDrivetrain swerve, ArmSubsystem arm, ShooterSubsystem shooter) {
        m_swerve = swerve;
        m_armSubsystem = arm;
        m_shooterSubsystem = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerve);

    }

    @Override
  public void initialize() {
    // Called when the command is initially scheduled.

  }

  @Override
  public void execute() {
    Pose2d waypoint1 = new Pose2d(1.46, 5.54, new Rotation2d(180));
    Pose2d waypoint2 = new Pose2d(3.31, 5.54, new Rotation2d(0));
    // Called every time Command is scheduled
    shootNote();
    moveArmtoIntake();
    
    m_swerve.followTrajectoryCommand(waypoint2);
  }

  @Override
  public void end(boolean interrupted) {
    //Called when command ends or is interrupted
  }

  @Override
  public boolean isFinished() {
    //Called when Command is finished
    return false;
  }

  private void shootNote() {
    m_armSubsystem.setArmAngle(Constants.Arm.launchAngle);

    if ( m_shooterSubsystem.isAtLaunchVelocity(Constants.Shooter.launchVelocity, Constants.Shooter.launchVelocityTolerance) && m_armSubsystem.isAtAngle(Constants.Arm.launchAngle, Constants.Arm.launchAngleTolerance)) {
       // m_armSubsystem.resetEncoders(Constants.Arm.launchAngle);
        
        m_shooterSubsystem.setFeedVelocity(Constants.Shooter.scoreSpeakerFeedVelocity);
        
        autoWait(1.5);
        m_shooterSubsystem.setLaunchVelocity(0);
        m_shooterSubsystem.setFeedVelocity(0);
        
    }

  }

  private void moveArmtoIntake() {
    m_armSubsystem.setArmAngle(Constants.Arm.intakeAngle);
    if (m_armSubsystem.isAtAngle(Constants.Arm.intakeAngle, Constants.Arm.intakeAngleTolerance) != true) {
        //m_armSubsystem.resetEncoders(Constants.Arm.intakeAngle);
            m_armSubsystem.setArmAngle(Constants.Arm.intakeAngle);
    
       }
  }
  
  private WaitCommand autoWait(double seconds) {
    WaitCommand shooterWait = new WaitCommand(seconds);

    return shooterWait;

  }
