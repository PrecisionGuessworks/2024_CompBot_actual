package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class ShootNoteSpeakerPodium extends Command{
    private final ShooterSubsystem m_shooterSubsystem;
    private final ArmSubsystem m_armSubsystem;
    private final IntakeSubsystem m_intake;
   
    private final Timer m_shotTimer = new Timer();

    public ShootNoteSpeakerPodium(ShooterSubsystem shooterSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intake) {
        m_shooterSubsystem = shooterSubsystem;
        m_armSubsystem = armSubsystem;
        m_intake = intake;
        //m_shotTimer.start();
        //m_armSubsystem = armSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooterSubsystem, armSubsystem, intake);

    }

    @Override
  public void initialize() {
    m_shotTimer.reset();
    //m_armSubsystem.setArmAngle(Constants.Arm.launchAngle);
    m_shooterSubsystem.setFeedVelocity(0);
    m_shooterSubsystem.setLaunchVelocity(Constants.Shooter.PodiumlaunchVelocity);

    // Called when the command is initially scheduled.
  }

  @Override
  public void execute() {
   
    
          
    
      
    m_armSubsystem.setArmAngle(Constants.Arm.PodiumlaunchAngle);

    if (m_shooterSubsystem.isAtLaunchVelocity(Constants.Shooter.PodiumlaunchVelocity, Constants.Shooter.PodiumlaunchVelocityTolerance) && m_armSubsystem.isAtAngle(Constants.Arm.PodiumlaunchAngle, Constants.Arm.PodiumlaunchAngleTolerance)) {
       // m_armSubsystem.resetEncoders(Constants.Arm.launchAngle);
       m_shooterSubsystem.setFeedVelocity(Constants.Shooter.scoreSpeakerFeedVelocity);
       m_shotTimer.start();
        
        
        
    }

    if (m_shotTimer.hasElapsed(0.3)) {
      m_shooterSubsystem.setFeedVelocity(0);
      m_shooterSubsystem.setLaunchVelocity(0);
      m_armSubsystem.setArmAngle(Constants.Arm.intakeAngle);
      
        
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
