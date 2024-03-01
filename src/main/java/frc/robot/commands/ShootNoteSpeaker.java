package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class ShootNoteSpeaker extends Command{
    private final ShooterSubsystem m_shooterSubsystem;
    private final ArmSubsystem m_armSubsystem;
    private boolean shoot = false;

    public ShootNoteSpeaker(ShooterSubsystem shooterSubsystem, ArmSubsystem armSubsystem) {
        m_shooterSubsystem = shooterSubsystem;
        m_armSubsystem = armSubsystem;
        //m_armSubsystem = armSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooterSubsystem, armSubsystem);

    }

    @Override
  public void initialize() {
    //m_armSubsystem.setArmAngle(Constants.Arm.launchAngle);
    m_shooterSubsystem.setFeedVelocity(0);
    m_shooterSubsystem.setLaunchVelocity(Constants.Shooter.launchVelocity);

    // Called when the command is initially scheduled.
  }

  @Override
  public void execute() {
    m_armSubsystem.setArmAngle(Constants.Arm.launchAngle);

    if ( m_shooterSubsystem.isAtLaunchVelocity(Constants.Shooter.launchVelocity, Constants.Shooter.launchVelocityTolerance) && m_armSubsystem.isAtAngle(Constants.Arm.launchAngle, Constants.Arm.launchAngleTolerance)) {
       // m_armSubsystem.resetEncoders(Constants.Arm.launchAngle);
        
        m_shooterSubsystem.setFeedVelocity(Constants.Shooter.scoreSpeakerFeedVelocity);
        if (m_shooterSubsystem.isAtFeedVelocity(Constants.Shooter.scoreSpeakerFeedVelocity, Constants.Shooter.launchVelocityTolerance)) {
          shoot = true;
        }
        
    }

    // Called every time Command is scheduled
  }

  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.setFeedVelocity(0);
    m_shooterSubsystem.setLaunchVelocity(0);
    //m_armSubsystem.setArmAngle((Constants.Arm.intakeAngle));

    //Called when command ends or is interrupted
  }

  @Override
  public boolean isFinished() {
    //Called when Command is finished
    
    return false;
  }
    


    
}
