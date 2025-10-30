package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ExampleSubsystem;


public class ExampleCommand extends Command{
    private final ExampleSubsystem m_subsystem;

    public ExampleCommand(ExampleSubsystem subsystem) {
        m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);

    }

    @Override
  public void initialize() {
    // Called when the command is initially scheduled.
  }

  @Override
  public void execute() {
    // Called every time Command is scheduled
  }

  @Override
  public void end(boolean interrupted) {
    //Called when command ends or is interrupted
  }

  @Override
  public boolean isFinished() {
    //Called when Command is finished
    return false;
  }
    


    
}
