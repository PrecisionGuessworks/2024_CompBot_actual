package frc.robot.commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class PanicIntakePiece extends Command{
    private final IntakeSubsystem m_intakeSubsystem;
    private final ShooterSubsystem m_shooter;
    private final ArmSubsystem m_arm;


    public PanicIntakePiece(IntakeSubsystem intake, ShooterSubsystem shooter, ArmSubsystem arm) {
        m_intakeSubsystem = intake;
        m_shooter = shooter;
        m_arm = arm;
        
    // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intake, shooter, arm);

    }

    @Override
  public void initialize() {
    //m_arm.setArmAngle(Constants.Arm.intakeAngle);
    m_intakeSubsystem.startRollerSpin();
    //m_shooter.Intake();
    

    // Called when the command is initially scheduled.
  }

  @Override
  public void execute() {
    
    

    m_arm.setArmAngle(Constants.Arm.intakeAngle);

    if (m_arm.isAtAngle(Constants.Arm.intakeAngle, Constants.Arm.intakeAngleTolerance)) {
      m_intakeSubsystem.startRollerSpin();
      m_shooter.Intake();
      
      if (m_intakeSubsystem.isRollerStalled()) {
      
      m_intakeSubsystem.stopRoller();
      m_shooter.stopRoller();
      
      
    }

    }
    // Called every time Command is scheduled
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stopRoller();
    m_shooter.stopRoller();
   
    //Called when command ends or is interrupted
  }

  @Override
  public boolean isFinished() {
    return (m_intakeSubsystem.isRollerStalled());
      
  }
    


    
}
