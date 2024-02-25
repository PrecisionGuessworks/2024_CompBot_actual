package frc.robot.commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ExampleSubsystem;


public class MoveClimber extends Command{
    private final ClimberSubsystem m_climber;
    private final XboxController m_xboxController;

    public MoveClimber(ClimberSubsystem subsystem, XboxController xboxController) {
        m_climber = subsystem;
        m_xboxController = xboxController;
    // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);

    }

    @Override
  public void initialize() {
    // Called when the command is initially scheduled.
  }

  @Override
  public void execute() {
    double m_rightOut = m_xboxController.getRightY();
    double m_leftOut = m_xboxController.getLeftY();
    //System.out.println("Climber right stick input"+ m_rightOut);
    //System.out.println("Climber left stick input"+ m_leftOut);
    
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
