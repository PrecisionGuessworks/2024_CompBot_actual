package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.motorcontrol.TalonFx;
import frc.robot.motorcontrol.configurations.TalonFxConfiguration;

public class ShooterSubsystem  extends SubsystemBase{

    

    private final TalonFx m_topMotor = new TalonFx(
      Constants.Shooter.TopRoller.topRollerID, Constants.Shooter.TopRoller.topRollerMotorRatio, 
      TalonFx.makeDefaultConfig().setInverted(Constants.Shooter.TopRoller.topRollerMotorInverted).setSupplyCurrentLimit(40.0)
      .setStatorCurrentLimit(40.0)
      .setPIDConfig(Constants.Shooter.TopRoller.topRollerMotorSlot,Constants.Shooter.TopRoller.topMotorPIDConfig)
    );

    private final TalonFx m_bottomMotor = new TalonFx(
      Constants.Shooter.BottomRoller.bottomRollerID, Constants.Shooter.BottomRoller.bottomRollerMotorRatio, 
      TalonFx.makeDefaultConfig().setInverted(Constants.Shooter.BottomRoller.bottomRollerMotorInverted).setSupplyCurrentLimit(40.0)
      .setStatorCurrentLimit(40.0)
      .setPIDConfig(Constants.Shooter.BottomRoller.bottomRollerMotorSlot,Constants.Shooter.BottomRoller.bottomMotorPIDConfig)
    );

    private final TalonFx m_feedMotor = new TalonFx(
      Constants.Shooter.Conveyer.conveyerID, Constants.Shooter.Conveyer.conveyerMotorRatio, 
      TalonFx.makeDefaultConfig().setInverted(Constants.Shooter.Conveyer.conveyerMotorInverted).setSupplyCurrentLimit(40.0)
      .setStatorCurrentLimit(40.0)
      .setPIDConfig(Constants.Shooter.Conveyer.conveyerRollerMotorSlot,Constants.Shooter.Conveyer.conveyerMotorPIDConfig)
    );

    public ShooterSubsystem() {
      //Body
      //Show scheduler status in SmartDashboard.
      SmartDashboard.putData(this);

    }

    public void setLaunchVelocity(double velocity) {
      if (velocity == 0.0) {
        m_topMotor.setPercentOutput(0.0);
        m_bottomMotor.setPercentOutput(0.0);
      } else {
        m_topMotor.setVelocitySetpoint(Constants.Shooter.TopRoller.topRollerMotorSlot, velocity);
        m_bottomMotor.setVelocitySetpoint(Constants.Shooter.BottomRoller.bottomRollerMotorSlot, velocity);
      }
    }

    public void setFeedVelocity(double velocity) {
      if (velocity == 0.0) {
        m_feedMotor.setPercentOutput(0.0);
      } else {
        m_feedMotor.setVelocitySetpoint(Constants.Shooter.Conveyer.conveyerRollerMotorSlot, velocity);
      }
    }

    public boolean isAtLaunchVelocity(double launchVelocity, double tolerance) {
      return Math.abs(launchVelocity - m_topMotor.getSensorVelocity()) <= tolerance
          && Math.abs(launchVelocity - m_bottomMotor.getSensorVelocity()) <= tolerance;
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
    

