package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.motorcontrol.TalonFx;
import frc.robot.motorcontrol.configurations.TalonFxConfiguration;

public class ExampleSubsystem  extends SubsystemBase{

    private final TalonFx m_motor = new TalonFx(
      Constants.Example.motorID, Constants.Example.motorRatio, TalonFx.makeDefaultConfig()
    );
    public ExampleSubsystem() {
      //Body
      //Show scheduler status in SmartDashboard.
      SmartDashboard.putData(this);

    }
    public void startSpin() {
      m_motor.setPercentOutput(1.0);
    }
  
    public void reverseSpin() {
      m_motor.setPercentOutput(-1.0);
    }
  
    public void stopSpin() {
      m_motor.setPercentOutput(0.0);
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
  
    // --- BEGIN STUFF FOR SIMULATION ---
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
    // --- END STUFF FOR SIMULATION ---
  }
    

