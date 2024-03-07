package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class ShootNoteSpeakerTogether extends Command{
    private final ShooterSubsystem m_shooterSubsystem;
    private final ArmSubsystem m_armSubsystem;
    private final IntakeSubsystem m_intake;
    int shottimeout = 0;
    boolean pastin = false;
    boolean isfirst = false;

    public ShootNoteSpeakerTogether(ShooterSubsystem shooterSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intake) {
        m_shooterSubsystem = shooterSubsystem;
        m_armSubsystem = armSubsystem;
        m_intake = intake;
        //m_armSubsystem = armSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooterSubsystem, armSubsystem, intake);

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
   
    if (!m_intake.isBeakBreakTriggered()) {
      if(pastin == true){
        shottimeout++;
        } 
      if (isfirst == false) {
        shottimeout = 0;
        isfirst = true;
      }
      
      if (shottimeout >= Constants.Arm.ShootTimeout){
          isfirst = false;
          pastin = false;
          m_shooterSubsystem.setFeedVelocity(0);
          m_shooterSubsystem.setLaunchVelocity(0);
          m_armSubsystem.setArmAngle(Constants.Arm.intakeAngle);
          }
          
    } else{
      pastin = true;
       m_armSubsystem.setArmAngle(Constants.Arm.launchAngle);

    if ( m_shooterSubsystem.isAtLaunchVelocity(Constants.Shooter.launchVelocity, Constants.Shooter.launchVelocityTolerance) && m_armSubsystem.isAtAngle(Constants.Arm.launchAngle, Constants.Arm.launchAngleTolerance)) {
       // m_armSubsystem.resetEncoders(Constants.Arm.launchAngle);
        
        m_shooterSubsystem.setFeedVelocity(Constants.Shooter.scoreSpeakerFeedVelocity);
        
        
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
