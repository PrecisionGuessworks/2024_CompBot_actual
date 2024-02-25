package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class EjectPiece extends Command{
    private final ShooterSubsystem m_shooter;
    private final ArmSubsystem m_arm;

    public EjectPiece(ShooterSubsystem shooter, ArmSubsystem arm) {
       m_shooter = shooter;
       m_arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooter, arm);

    }

    @Override
  public void initialize() {
    
    // Called when the command is initially scheduled.
  }

  @Override
  public void execute() {
    
    m_arm.setArmAngle(Constants.Arm.eject);

    if (m_arm.isAtAngle(Constants.Arm.eject, Constants.Arm.ejectAngleTolerance)) {
       // m_armSubsystem.resetEncoders(Constants.Arm.launchAngle);
        
        m_shooter.eject();
    // Called every time Command is scheduled
  }}

  @Override
  public void end(boolean interrupted) {
    //Called when command ends or is interrupted
    m_shooter.stopRoller();
  }

  @Override
  public boolean isFinished() {
    //Called when Command is finished
    return false;
  }
    


    
}
