package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class IntakePiece extends Command{
    private final IntakeSubsystem m_intakeSubsystem;
    private final ShooterSubsystem m_shooter;

    public IntakePiece(IntakeSubsystem intake, ShooterSubsystem shooter) {
        m_intakeSubsystem = intake;
        m_shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intake, shooter);

    }

    @Override
  public void initialize() {
    m_intakeSubsystem.startRollerSpin();
    m_shooter.Intake();

    // Called when the command is initially scheduled.
  }

  @Override
  public void execute() {
    m_intakeSubsystem.startRollerSpin();
    m_shooter.Intake();
    if (m_intakeSubsystem.isRollerStalled() || m_intakeSubsystem.isBeakBreakTriggered()) {
      
      m_intakeSubsystem.stopRoller();
      m_shooter.stopRoller();
      
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
    if (m_intakeSubsystem.isRollerStalled() || m_intakeSubsystem.isBeakBreakTriggered()) {
      m_intakeSubsystem.stopRoller();
      m_shooter.stopRoller();
      
      return true;
      
    }
    
    //Called when Command is finished
    return false;
  }
    


    
}
