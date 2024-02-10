package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


public class IntakePiece extends Command{
    private final IntakeSubsystem m_intakeSubsystem;

    public IntakePiece(IntakeSubsystem subsystem) {
        m_intakeSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);

    }

    @Override
  public void initialize() {
    m_intakeSubsystem.startRollerSpin();
    // Called when the command is initially scheduled.
  }

  @Override
  public void execute() {
    m_intakeSubsystem.startRollerSpin();
    if (m_intakeSubsystem.isRollerStalled() == true) {
      
      m_intakeSubsystem.stopRoller();
      
    }
    
    // Called every time Command is scheduled
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stopRoller();
    //Called when command ends or is interrupted
  }

  @Override
  public boolean isFinished() {
    if (m_intakeSubsystem.isRollerStalled() == true) {
      
      m_intakeSubsystem.stopRoller();
      return true;
      
    }
    
    //Called when Command is finished
    return false;
  }
    


    
}
