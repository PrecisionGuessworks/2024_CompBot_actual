package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


public class ScoreAmp extends Command{
    private final ShooterSubsystem m_shooterSubsystem;
    private final ArmSubsystem m_armSubsystem;
    private final IntakeSubsystem m_intake;
    int shottimeout = 0;
    boolean pastin = false;
    boolean isfirst = false;
    boolean second = false;
    private final Timer m_shotTimer = new Timer();

    public ScoreAmp(ShooterSubsystem shooter, ArmSubsystem arm , IntakeSubsystem intake) {
        m_shooterSubsystem = shooter;
        m_armSubsystem = arm;
        m_intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooter, arm, intake);

    }

    @Override
  public void initialize() {
    // Called when the command is initially scheduled.
    m_shotTimer.reset();
    m_shooterSubsystem.setFeedVelocity(0);
    m_shooterSubsystem.spinAmp(Constants.Shooter.ScoreAmpPower);
    
  }

  @Override
  public void execute() {
    // Called every time Command is scheduled

     if (m_shotTimer.hasElapsed(1.5)) {
          m_shotTimer.stop();
        }
      else if (m_shotTimer.hasElapsed(Constants.Arm.AmpTimeoutMid)) {
        m_armSubsystem.setArmAngle(Constants.Arm.intakeAngle);
        //m_shotTimer.reset();

       
      }
      else if (m_shotTimer.hasElapsed(Constants.Arm.AmpTimeout)) {
      m_armSubsystem.setArmAngle(Constants.Arm.ampPreAngle);
      m_shooterSubsystem.setFeedVelocity(0);
      m_shooterSubsystem.setLaunchVelocity(0);

      
    }

    else {
       m_armSubsystem.setArmAngle(Constants.Arm.scoreAmpArmAngle);
    if (m_armSubsystem.isAtAngle(Constants.Arm.scoreAmpArmAngle, Constants.Arm.scoreAmpArmAngleTolerance)) {
       // m_armSubsystem.resetEncoders(Constants.Arm.scoreAmpArmAngle);      
        m_shooterSubsystem.setFeedVelocity(Constants.Shooter.scoreAmpFeedVelocity);
        m_shotTimer.start();   
        
    } 

    }
  }

  @Override
  public void end(boolean interrupted) {
    //Called when command ends or is interrupted
    m_shooterSubsystem.setFeedVelocity(0);
    m_shooterSubsystem.spinAmp(0);
    m_armSubsystem.setArmAngle((Constants.Arm.intakeAngle));
  }

  @Override
  public boolean isFinished() {
    //Called when Command is finished
    //return m_armSubsystem.isArmMotionFinished();
    return false;
  }
    


    
}