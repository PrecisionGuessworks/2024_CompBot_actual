package frc.robot.motorcontrol.configurations;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.motorcontrol.devices.EasyStatusSignal;
import frc.robot.motorcontrol.configurations.phoenix.PhoenixUtils;
import frc.robot.Robot;
import java.util.function.Function;



import frc.robot.motorcontrol.PIDConfig;

public class TalonFxConfiguration {
    private NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
    private boolean ENABLE_VOLTAGE_COMPENSATION = true;
    private double VOLTAGE_COMPENSATION_SATURATION = 12.0; // V
    private double NEUTRAL_DEADBAND = 0.001; // 0.1 %
    public boolean INVERTED = false;
    private boolean ENABLE_CURRENT_LIMIT = true;
    public double SUPPLY_CURRENT_LIMIT = 40.0; // A
    public double STATOR_CURRENT_LIMIT = 40.0; // A
    public double CURRENT_LIMIT_TRIGGER_THRESHOLD_TIME = 0.1; // s
    private boolean FWD_SOFT_LIMIT_ENABLED = false;
    private double FWD_SOFT_LIMIT = 0.0; // In MechanismRatio units
    private boolean REV_SOFT_LIMIT_ENABLED = false;
    private double REV_SOFT_LIMIT = 0.0; // In MechanismRatio units
    private PIDConfig slot0Config = new PIDConfig();
    public PIDConfig slot1Config = new PIDConfig();
    public PIDConfig slot2Config = new PIDConfig();
    public double sensorUpdateFrequencyHz = 100.0;
    public boolean fusedCANcoder = false;
    public int fusedCANcodeID = 99;

    public TalonFxConfiguration setBrakeMode() {
      NEUTRAL_MODE = NeutralModeValue.Brake;
      return this;
    }

    public TalonFxConfiguration setInverted(final boolean inverted) {
      INVERTED = inverted;
      return this;
    }

    public TalonFxConfiguration setStatorCurrentLimit(final double amps) {
      STATOR_CURRENT_LIMIT = amps;
      return this;
    }

    public TalonFxConfiguration setSupplyCurrentLimit(final double amps) {
      SUPPLY_CURRENT_LIMIT = amps;
      return this;
    }
    public TalonFxConfiguration setForwardSoftLimit(final double pos) {
        FWD_SOFT_LIMIT_ENABLED = true;
        FWD_SOFT_LIMIT = pos;
        return this;
      }
  
      public TalonFxConfiguration setReverseSoftLimit(final double pos) {
        REV_SOFT_LIMIT_ENABLED = true;
        REV_SOFT_LIMIT = pos;
        return this;
      }
      public TalonFxConfiguration setPIDConfig(final int slot, final PIDConfig config) {
        switch (slot) {
          case 0:
            slot0Config = config;
            break;
          case 1:
            slot1Config = config;
            break;
          case 2:
            slot2Config = config;
            break;
          default:
            throw new RuntimeException("Invalid PID slot " + slot);
        }
        return this;
      }

      /*public TalonFxConfiguration setFusedCANCoder(canCoder) {
        fusedCANcoder = true;
        fusedCANcodeID = canCoder.getDeviceID();


        return this;


      } */

      public TalonFXConfiguration toTalonFXConfiguration(
        final Function<Double, Double> toNativeSensorPosition) {
      final TalonFXConfiguration config = new TalonFXConfiguration();
      config.MotorOutput.NeutralMode = NEUTRAL_MODE;
      config.MotorOutput.Inverted =
          INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
      config.MotorOutput.DutyCycleNeutralDeadband = 0.0;

      if (Robot.isReal()) {
        
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
      }
      config.CurrentLimits.SupplyCurrentLimitEnable = true;
      config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
      config.CurrentLimits.SupplyCurrentThreshold = SUPPLY_CURRENT_LIMIT;
      config.CurrentLimits.SupplyTimeThreshold = 0.1; // s

      config.SoftwareLimitSwitch.ForwardSoftLimitEnable = FWD_SOFT_LIMIT_ENABLED;
      config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
          toNativeSensorPosition.apply(FWD_SOFT_LIMIT);
      config.SoftwareLimitSwitch.ReverseSoftLimitEnable = REV_SOFT_LIMIT_ENABLED;
      config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
          toNativeSensorPosition.apply(REV_SOFT_LIMIT);

      config.Voltage.SupplyVoltageTimeConstant = 0.0;
      config.Voltage.PeakForwardVoltage = 16.0;
      config.Voltage.PeakReverseVoltage = 16.0;

      config.Slot0 = slot0Config.fillCTRE(new Slot0Configs());
      config.Slot1 = slot1Config.fillCTRE(new Slot1Configs());
      config.Slot2 = slot2Config.fillCTRE(new Slot2Configs());
      //config.Feedback.FeedbackRemoteSensorID = ;
      //config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
      return config;
    }

    
}
