package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmIntakeAmp extends Command{
    private final ArmSubsystem m_armSubsystem;
    private boolean atMidpoint = false;

    public MoveArmIntakeAmp(ArmSubsystem subsystem) {
        m_armSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);

    }

    @Override
  public void initialize() {
    
    // Called when the command is initially scheduled.
  }

  @Override
  public void execute() {
    
    if (m_armSubsystem.getArmAngle() >= (Constants.Arm.midpointAngle+Constants.Arm.midpointAngleTolerance)) {
      m_armSubsystem.setArmAngle(Constants.Arm.midpointAngle);
      

    }
    else {
      if (((m_armSubsystem.getArmAngle()+Constants.Arm.midpointAngleTolerance) <= Constants.Arm.midpointAngle)) {
      m_armSubsystem.setArmAngle(Constants.Arm.intakeAngle);

   }

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
    //return m_armSubsystem.isArmMotionFinished();
    return false;
  }
    


    
}
