package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.motorcontrol.TalonFx;
import frc.robot.motorcontrol.configurations.TalonFxConfiguration;

public class IntakeSubsystem  extends SubsystemBase{

    private final TalonFx m_rollerMotor = new TalonFx(
      Constants.Intake.Roller.rollerMotorID, 
      Constants.Intake.Roller.rollerMotorRatio,
      TalonFx.makeDefaultConfig().setInverted(Constants.Intake.Roller.rollerMotorInverted)
    );

    private final Timer m_rollerTimer = new Timer();

    public IntakeSubsystem() {
      m_rollerTimer.start();
      //Show scheduler status in SmartDashboard.
      SmartDashboard.putData(this);

    }

    public void startRollerSpin() {
        m_rollerMotor.setCurrentLimit(20.0);
        m_rollerMotor.setPercentOutput(Constants.Intake.Roller.rollerIntakePower);
        m_rollerTimer.reset();

    }

    public void stopRoller() {
        m_rollerMotor.setCurrentLimit(20.0);
        m_rollerMotor.setPercentOutput(0.0);
        m_rollerTimer.reset();
      }

      public void spinRollerSlow() {
        m_rollerMotor.setCurrentLimit(20.0);
        m_rollerMotor.setPercentOutput(Constants.Intake.Roller.rollerSlowPower);
        m_rollerTimer.reset();
      }

    public boolean isRollerStalled() {
        if (Math.abs(m_rollerMotor.getSensorVelocity()) > Constants.Intake.Roller.rollerStallSpeed) {
            m_rollerTimer.reset();
            return false;
          }
          return m_rollerTimer.get() > Constants.Intake.Roller.rollerStallTime;
    }

    
    

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Roller: Current", m_rollerMotor.getStatorCurrent());
      // This method will be called once per scheduler run
    }
  
    // --- BEGIN STUFF FOR SIMULATION ---
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
    // --- END STUFF FOR SIMULATION ---
  }
    


