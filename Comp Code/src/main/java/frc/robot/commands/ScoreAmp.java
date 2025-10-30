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
    m_shooterSubsystem.setFeedVelocity(0);
    m_shooterSubsystem.spinAmp(Constants.Shooter.ScoreAmpPower);
  }

  @Override
  public void execute() {
    // Called every time Command is scheduled

  

    if (!m_intake.isBeamBreakTriggered()) {
      
      shottimeout++;
           
      
      if ((shottimeout >= Constants.Arm.AmpTimeout) && (shottimeout < Constants.Arm.AmpTimeoutMid)){
        isfirst = false;
        m_shooterSubsystem.setFeedVelocity(0);
        m_shooterSubsystem.setLaunchVelocity(0);
        m_armSubsystem.setArmAngle(Constants.Arm.moveAmpArmAngle);
        } 
        if (shottimeout >= Constants.Arm.AmpTimeoutMid){
        isfirst = false;
        second = false;
        pastin = false;
        m_shooterSubsystem.setFeedVelocity(0);
        m_shooterSubsystem.setLaunchVelocity(0);
        m_armSubsystem.setArmAngle(Constants.Arm.intakeAngle);   
        }     
           
    } else{
    
    shottimeout = 0;
    m_armSubsystem.setArmAngle(Constants.Arm.scoreAmpArmAngle);
    if (m_armSubsystem.isAtAngle(Constants.Arm.scoreAmpArmAngle, Constants.Arm.scoreAmpArmAngleTolerance)) {
       // m_armSubsystem.resetEncoders(Constants.Arm.scoreAmpArmAngle);      
        m_shooterSubsystem.setFeedVelocity(Constants.Shooter.scoreAmpFeedVelocity);   
        
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
