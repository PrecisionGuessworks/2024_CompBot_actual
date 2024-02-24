package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;
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

    private final TalonFx m_ampMotor = new TalonFx(
      Constants.Shooter.Amp.ampID, Constants.Shooter.Amp.ampMotorRatio, 
      TalonFx.makeDefaultConfig().setInverted(Constants.Shooter.Amp.ampMotorInverted).setSupplyCurrentLimit(40.0)
      .setStatorCurrentLimit(40.0)
      .setPIDConfig(Constants.Shooter.Amp.ampRollerMotorSlot,Constants.Shooter.Amp.ampMotorPIDConfig)
    );

    private final SimpleMotorFeedforward m_topFF = Constants.Shooter.TopRoller.topRollerFeedforward;
    private final SimpleMotorFeedforward m_bottomFF = Constants.Shooter.BottomRoller.bottomRollerFeedforward;
    private final SimpleMotorFeedforward m_feedFF = Constants.Shooter.Conveyer.feedFeedforward;
    private final SimpleMotorFeedforward m_ampFF = Constants.Shooter.Amp.ampFeedforward;

    private final Timer m_rollerTimer = new Timer();

    public ShooterSubsystem() {
      m_rollerTimer.start();
      //Body
      //Show scheduler status in SmartDashboard.
      SmartDashboard.putData(this);

    }

    public void setLaunchVelocity(double velocity) {
      final double topFFVolts = m_topFF.calculate(velocity);
      final double bottomFFVolts = m_bottomFF.calculate(velocity);

      if (velocity == 0.0) {
        m_topMotor.setPercentOutput(0.0);
        m_bottomMotor.setPercentOutput(0.0);
      } else {
        m_topMotor.setVelocitySetpoint(Constants.Shooter.TopRoller.topRollerMotorSlot, velocity, topFFVolts);
        m_bottomMotor.setVelocitySetpoint(Constants.Shooter.BottomRoller.bottomRollerMotorSlot, velocity, bottomFFVolts);
      }
    }

    public void setAmpVelocity(double velocity) {
      final double topFFVolts = m_topFF.calculate(-velocity);
      final double bottomFFVolts = m_bottomFF.calculate(velocity);

      if (velocity == 0.0) {
        m_topMotor.setPercentOutput(0.0);
        m_bottomMotor.setPercentOutput(0.0);
      } else {
        m_topMotor.setVelocitySetpoint(Constants.Shooter.TopRoller.topRollerMotorSlot, velocity, topFFVolts);
        m_bottomMotor.setVelocitySetpoint(Constants.Shooter.BottomRoller.bottomRollerMotorSlot, velocity, bottomFFVolts);
      }
    }

    public void setFeedVelocity(double velocity) {
      final double feedFFVolts = m_feedFF.calculate(velocity);
      final double ampFFVolts = m_ampFF.calculate(velocity);

      if (velocity == 0.0) {
        m_feedMotor.setPercentOutput(0.0);
        m_ampMotor.setPercentOutput(0.0);
      } else {
        m_feedMotor.setVelocitySetpoint(Constants.Shooter.Conveyer.conveyerRollerMotorSlot, velocity, feedFFVolts);
        m_ampMotor.setVelocitySetpoint(Constants.Shooter.Amp.ampRollerMotorSlot, velocity, ampFFVolts);
      }
    }

    public boolean isAtLaunchVelocity(double launchVelocity, double tolerance) {
      return Math.abs(launchVelocity - m_topMotor.getSensorVelocity()) <= tolerance
          && Math.abs(launchVelocity - m_bottomMotor.getSensorVelocity()) <= tolerance;
    }

    public boolean isAtAmpVelocity(double launchVelocity, double tolerance) {
      return Math.abs(launchVelocity - Math.abs(m_topMotor.getSensorVelocity())) <= tolerance
          && Math.abs(launchVelocity - Math.abs(m_bottomMotor.getSensorVelocity())) <= tolerance;
    }

    public void Intake() {
      m_topMotor.setCurrentLimit(40.0);
      m_bottomMotor.setCurrentLimit(40.0);
      m_topMotor.setPercentOutput(Constants.Shooter.TopRoller.topIntakePower);
      m_bottomMotor.setPercentOutput(Constants.Shooter.BottomRoller.bottomIntakePower);
      m_ampMotor.setCurrentLimit(40.0);
      m_feedMotor.setCurrentLimit(40.0);
      m_feedMotor.setPercentOutput(Constants.Shooter.Conveyer.conveyerIntakePower);
      m_ampMotor.setPercentOutput(Constants.Shooter.Amp.ampIntakePower);
      m_rollerTimer.reset();



  }

  public void stopRoller() {
      m_feedMotor.setPercentOutput(0.0);
      m_ampMotor.setPercentOutput(0.0);
      m_topMotor.setPercentOutput(0.0);
      m_bottomMotor.setPercentOutput(0.0);
      m_rollerTimer.reset();
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
    

