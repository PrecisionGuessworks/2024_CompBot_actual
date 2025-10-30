package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmSpeaker extends Command{
    private final ArmSubsystem m_armSubsystem;

    public MoveArmSpeaker(ArmSubsystem subsystem) {
        m_armSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);

    }

    @Override
  public void initialize() {
    m_armSubsystem.setArmAngle(Constants.Arm.launchAngle);
    // Called when the command is initially scheduled.
  }

  @Override
  public void execute() {
   if (m_armSubsystem.isAtAngle(Constants.Arm.launchAngle, Constants.Arm.launchAngleTolerance) != true) {
        m_armSubsystem.setArmAngle(Constants.Arm.launchAngle);

   }
    // Called every time Command is scheduled
  }

  @Override
  public void end(boolean interrupted) {
    //Called when command ends or is interrupted
  }

  @Override
  public boolean isFinished() {
    //Called when Command is finished
    return m_armSubsystem.isArmMotionFinished();
    
  }
    


    
}
