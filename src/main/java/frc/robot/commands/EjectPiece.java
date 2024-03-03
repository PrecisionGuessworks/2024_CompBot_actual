package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class EjectPiece extends Command{
    private final ShooterSubsystem m_shooter;
    private final ArmSubsystem m_arm;
    private final IntakeSubsystem m_intake;

    public EjectPiece(ShooterSubsystem shooter, ArmSubsystem arm, IntakeSubsystem intake) {
       m_shooter = shooter;
       m_arm = arm;
       m_intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooter, arm, intake);

    }

    @Override
  public void initialize() {
    m_intake.reverseRollerSpin();
    
    // Called when the command is initially scheduled.
  }

  @Override
  public void execute() {
    
    m_arm.setArmAngle(Constants.Arm.eject);

    if (m_arm.isAtAngle(Constants.Arm.eject, Constants.Arm.ejectAngleTolerance)) {
       // m_armSubsystem.resetEncoders(Constants.Arm.launchAngle);
        
        m_shooter.eject();
        m_intake.reverseRollerSpin();
    // Called every time Command is scheduled
  }}

  @Override
  public void end(boolean interrupted) {
    //Called when command ends or is interrupted
    m_shooter.stopRoller();
    m_intake.stopRoller();
  }

  @Override
  public boolean isFinished() {
    //Called when Command is finished
    return false;
  }
    


    
}
