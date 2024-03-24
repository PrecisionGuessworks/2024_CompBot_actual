package frc.robot.commands;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;


public class TurnTest extends Command{
    private final CommandSwerveDrivetrain m_swerve;
    private final SwerveRequest.FieldCentric m_drive;
    private XboxController m_joystick;

    private final double MaxAngularRate = 2 * Math.PI;
    private final double MaxSpeed = 5.0292; 

    public TurnTest(CommandSwerveDrivetrain swerve,SwerveRequest.FieldCentric drive, XboxController joystick) {
        m_swerve = swerve;
        m_drive = drive;
        m_joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerve);

    }

    @Override
  public void initialize() {
    // Called when the command is initially scheduled.
  }

  @Override
  public void execute() {
    Supplier<SwerveRequest> requestSupplier =  () -> m_drive.withVelocityX(-m_joystick.getLeftY() * MaxSpeed).withVelocityY(-m_joystick.getLeftX() * MaxSpeed).withRotationalRate(0.4*MaxAngularRate);
    m_swerve.setControl(requestSupplier.get());
    

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
