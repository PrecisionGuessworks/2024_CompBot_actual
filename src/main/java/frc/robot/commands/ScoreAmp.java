package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class ScoreAmp extends Command{
    private final ShooterSubsystem m_shooterSubsystem;
    private final ArmSubsystem m_armSubsystem;

    public ScoreAmp(ShooterSubsystem shooter, ArmSubsystem arm) {
        m_shooterSubsystem = shooter;
        m_armSubsystem = arm;
    // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooter, arm);

    }

    @Override
  public void initialize() {
    // Called when the command is initially scheduled.
    m_shooterSubsystem.setFeedVelocity(0);
    m_shooterSubsystem.spinAmp(Constants.Shooter.ScoreAmpPower);
  }

  @Override
  public void execute() {
    // Called every time Command is scheduled
    
    m_armSubsystem.setArmAngle(Constants.Arm.scoreAmpArmAngle);
    if (m_armSubsystem.isAtAngle(Constants.Arm.scoreAmpArmAngle, Constants.Arm.scoreAmpArmAngleTolerance)) {
        m_shooterSubsystem.setFeedVelocity(Constants.Shooter.scoreAmpFeedVelocity);
        
        
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
