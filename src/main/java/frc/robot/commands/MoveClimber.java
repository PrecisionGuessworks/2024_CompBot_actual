package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ExampleSubsystem;


public class MoveClimber extends Command{
    private final ClimberSubsystem m_climber;
    private final double m_rightOut;
    private final double m_leftOut;

    public MoveClimber(ClimberSubsystem subsystem, double rightOut, double leftOut) {
        m_climber = subsystem;
        m_rightOut = rightOut;
        m_leftOut = leftOut;
    // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);

    }

    @Override
  public void initialize() {
    // Called when the command is initially scheduled.
  }

  @Override
  public void execute() {
    
    m_climber.moveClimber(m_rightOut, m_leftOut);
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
