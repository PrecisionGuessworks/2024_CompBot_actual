package frc.robot.commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class IntakePiece extends Command{
    private final IntakeSubsystem m_intakeSubsystem;
    private final ShooterSubsystem m_shooter;
    private final ArmSubsystem m_arm;
    private final XboxController m_xboxController;

    public IntakePiece(IntakeSubsystem intake, ShooterSubsystem shooter, ArmSubsystem arm, XboxController xboxController) {
        m_intakeSubsystem = intake;
        m_shooter = shooter;
        m_arm = arm;
        m_xboxController = xboxController;
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
      
      if (m_intakeSubsystem.isRollerStalled() || m_intakeSubsystem.isBeakBreakTriggered()) {
      
      m_intakeSubsystem.stopRoller();
      m_shooter.stopRoller();
      m_xboxController.setRumble(RumbleType.kBothRumble, 1);
      
    }

    }
    // Called every time Command is scheduled
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stopRoller();
    m_shooter.stopRoller();
    m_xboxController.setRumble(RumbleType.kBothRumble, 0);
    //Called when command ends or is interrupted
  }

  @Override
  public boolean isFinished() {
    if (m_intakeSubsystem.isRollerStalled() || m_intakeSubsystem.isBeakBreakTriggered()) {
      m_intakeSubsystem.stopRoller();
      m_shooter.stopRoller();
      
      return true;
      
    }
    
    //Called when Command is finished
    return false;
  }
    


    
}
