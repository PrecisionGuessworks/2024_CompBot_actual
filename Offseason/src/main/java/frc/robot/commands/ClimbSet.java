package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbSet extends Command {

  private final ClimberSubsystem m_climber;
  private final ArmSubsystem m_arm;
  //private Timer m_ejectTimer = new Timer();
  private int m_climbPosition = 0;

  public ClimbSet(int climbPosition, ClimberSubsystem climber, ArmSubsystem arm) {
    m_climbPosition = climbPosition;
    m_climber = climber;
    m_arm = arm;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_climbPosition == 1){
    m_climber.setHeight(Constants.Climber.stowHeight);
    } else if (m_climbPosition == 2){
      m_climber.setHeight(Constants.Climber.upperStowHeight);
    } else if (m_climbPosition == 3){
      m_climber.setHeight(Constants.Climber.climbHeight);
    } else {
      m_climber.setHeight(Constants.Climber.stowHeight);
    }

    
    // m_intake.setRollerVelocity(-1.0);
    // m_intake.setAngle(Constants.Intake.intakeClimbAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_climber.setHeight(Constants.Climber.L2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_climber.setHeight(Constants.Climber.stowHeight);
  }

  // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return m_ejectTimer.get() > 0.35;
  // }
}
