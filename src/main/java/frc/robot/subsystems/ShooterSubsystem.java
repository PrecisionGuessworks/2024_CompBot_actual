package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.motorcontrol.TalonFx;
import frc.robot.motorcontrol.configurations.TalonFxConfiguration;

public class ShooterSubsystem  extends SubsystemBase{

    private final TalonFx m_topMotor = new TalonFx(
      Constants.Example.motorID, Constants.Example.motorRatio, TalonFx.makeDefaultConfig()
    );
    public ShooterSubsystem() {
      //Body
      //Show scheduler status in SmartDashboard.
      SmartDashboard.putData(this);

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
    

