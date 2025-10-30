package frc.robot.motorcontrol;

import com.ctre.phoenix6.StatusSignal;
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
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.motorcontrol.configurations.TalonFxConfiguration;
import frc.robot.motorcontrol.devices.CANDeviceID;
import frc.robot.motorcontrol.devices.EasyStatusSignal;
import frc.robot.motorcontrol.configurations.phoenix.PhoenixUtils;

import java.util.function.Function;

public class TalonFx {
  private static final double kCANTimeoutS = 0.1;
  private static final double kDefaultUpdateFreqHz = 100.0;
  private final CANDeviceID m_canID;
  public final TalonFX m_controller;
  //private final TalonFXSimState m_simState;
  //ToDo: sim stuff
  private final MechanismRatio m_ratio;
  private final TalonFxConfiguration m_config;

  private final DutyCycleOut m_dutyCycleControl = new DutyCycleOut(0);
  private final VoltageOut m_voltageControl = new VoltageOut(0);
  private final VelocityVoltage m_velocityControl = new VelocityVoltage(0);
  private final PositionVoltage m_positionControl = new PositionVoltage(0);
  private final EasyStatusSignal m_percentOutputSignal;
  private final EasyStatusSignal m_sensorPositionSignal;
  private final EasyStatusSignal m_sensorVelocitySignal;
  private double sensorUpdateFrequencyHz = 100.0;
  
  public TalonFx(
      final CANDeviceID canID,
      final TalonFx leader,
      final boolean opposeLeader,
      final TalonFxConfiguration config) {
    this(canID, leader.getMechanismRatio(), config);
    m_controller.setControl(new Follower(leader.getDeviceID(), opposeLeader));
  }

    public TalonFx(
      final CANDeviceID canID, final MechanismRatio ratio, final TalonFxConfiguration config) {
        m_canID = canID;
        m_controller = new TalonFX(canID.deviceNumber, canID.CANbusName);
        //m_simState = m_controller.getSimState();
        m_ratio = ratio;
        m_config = config;
    
        m_percentOutputSignal = new EasyStatusSignal(m_controller.getDutyCycle());
        m_sensorPositionSignal =
            new EasyStatusSignal(m_controller.getRotorPosition(), this::fromNativeSensorPosition);
        m_sensorVelocitySignal =
            new EasyStatusSignal(m_controller.getRotorVelocity(), this::fromNativeSensorVelocity);
    
        // Clear reset flag.
        m_controller.hasResetOccurred();
    
        setConfiguration();
  }
  public void setConfiguration() {
    // Set motor controller configuration.
    final TalonFXConfiguration config =
        m_config.toTalonFXConfiguration(this::toNativeSensorPosition);
    PhoenixUtils.retryUntilSuccess(
        () -> m_controller.getConfigurator().apply(config, kCANTimeoutS),
        () -> {
          TalonFXConfiguration readConfig = new TalonFXConfiguration();
          m_controller.getConfigurator().refresh(readConfig, kCANTimeoutS);
          return PhoenixUtils.TalonFXConfigsEqual(config, readConfig);
        },
        "TalonFX " + m_canID + ": applyConfiguration");

    // Set update frequencies.
    PhoenixUtils.retryUntilSuccess(
        () ->
            m_percentOutputSignal.setUpdateFrequency(
                m_config.sensorUpdateFrequencyHz, kCANTimeoutS),
        () -> m_percentOutputSignal.getAppliedUpdateFrequency() == m_config.sensorUpdateFrequencyHz,
        "TalonFX " + m_canID + ": m_percentOutputSignal.setUpdateFrequency()");
    PhoenixUtils.retryUntilSuccess(
        () ->
            m_sensorPositionSignal.setUpdateFrequency(
                m_config.sensorUpdateFrequencyHz, kCANTimeoutS),
        () ->
            m_sensorPositionSignal.getAppliedUpdateFrequency() == m_config.sensorUpdateFrequencyHz,
        "TalonFX " + m_canID + ": m_sensorPositionSignal.setUpdateFrequency()");
    PhoenixUtils.retryUntilSuccess(
        () ->
            m_sensorVelocitySignal.setUpdateFrequency(
                m_config.sensorUpdateFrequencyHz, kCANTimeoutS),
        () ->
            m_sensorVelocitySignal.getAppliedUpdateFrequency() == m_config.sensorUpdateFrequencyHz,
        "TalonFX " + m_canID + ": m_sensorVelocitySignal.setUpdateFrequency()");

    // Disable all signals that have not been explicitly defined.
    PhoenixUtils.retryUntilSuccess(
        () -> m_controller.optimizeBusUtilization(kCANTimeoutS),
        "TalonFX " + m_canID + ": optimizeBusUtilization");

    // Block until we get valid signals.
    PhoenixUtils.retryUntilSuccess(
        () ->
            EasyStatusSignal.waitForAll(
                kCANTimeoutS,
                m_percentOutputSignal,
                m_sensorPositionSignal,
                m_sensorVelocitySignal),
        "TalonFX " + m_canID + ": waitForAll()");
  }

  public boolean checkFaultsAndReconfigureIfNecessary() {
    // TODO: Log other faults.
    if (m_controller.hasResetOccurred()) {
      DriverStation.reportError("TalonFX " + m_canID + ": reset occured", false);
      setConfiguration();
      return true;
    }
    return false;
  }

  public static TalonFxConfiguration makeDefaultConfig() {
    return new TalonFxConfiguration();
  }
  public double toNativeSensorPosition(final double pos) {
    return toNativeSensorPosition(pos, m_ratio);
  }

  public static double toNativeSensorPosition(final double pos, final MechanismRatio mr) {
    // Native position is rotations. There is 1 rotation per revolution (lol).
    return mr.mechanismPositionToSensorRadians(pos) / (2.0 * Math.PI);
  }

  public double fromNativeSensorPosition(final double pos) {
    return pos / toNativeSensorPosition(1.0);
  }

  public double toNativeSensorVelocity(final double vel) {
    return toNativeSensorVelocity(vel, m_ratio);
  }

  public static double toNativeSensorVelocity(final double vel, final MechanismRatio mr) {
    // Native velocity is rotations per second.
    return toNativeSensorPosition(vel, mr);
  }

  public double fromNativeSensorVelocity(final double vel) {
    return vel / toNativeSensorVelocity(1.0);
  }

  public EasyStatusSignal percentOutputSignal() {
    return m_percentOutputSignal;
  }

  public EasyStatusSignal sensorPositionSignal() {
    return m_sensorPositionSignal;
  }

  public EasyStatusSignal sensorVelocitySignal() {
    return m_sensorVelocitySignal;
  }

  public void close() {
    m_controller.close();
  }

  public int getDeviceID() {
    return m_controller.getDeviceID();
  }

  public void setCurrentLimit(final double amps) {
    m_config.SUPPLY_CURRENT_LIMIT = amps;
    //m_config.STATOR_CURRENT_LIMIT = amps;

    // TODO: Consider a shorter non-blocking timeout
    m_controller
        .getConfigurator()
        .apply(
            m_config.toTalonFXConfiguration(this::toNativeSensorPosition).CurrentLimits,
            kCANTimeoutS);
  }
  public void setPercentOutput(final double percent) {
    m_dutyCycleControl.Output = percent;
    m_controller.setControl(m_dutyCycleControl);
  }

  public void setVoltageOutput(final double voltage) {
    setVoltageOutput(voltage, false);
  }

  public void setVoltageOutput(final double voltage, final boolean synchronous) {
    m_voltageControl.UpdateFreqHz = synchronous ? 0.0 : kDefaultUpdateFreqHz;
    m_voltageControl.Output = voltage;
    m_controller.setControl(m_voltageControl);
  }

  public void setPositionSetpoint(final int slot, final double setpoint) {
    setPositionSetpoint(slot, setpoint, 0.0);
  }

  public void setPositionSetpoint(
      final int slot, final double setpoint, final boolean synchronous) {
    setPositionSetpoint(slot, setpoint, 0.0, synchronous);
  }

  public void setPositionSetpoint(
      final int slot, final double setpoint, final double feedforwardVolts) {
    setPositionSetpoint(slot, setpoint, feedforwardVolts, false);
  }
  public void setPositionSetpoint(
    final int slot,
    final double setpoint,
    final double feedforwardVolts,
    final boolean synchronous) {
  m_positionControl.UpdateFreqHz = synchronous ? 0.0 : kDefaultUpdateFreqHz;
  m_positionControl.Slot = slot;
  m_positionControl.Position = toNativeSensorPosition(setpoint);
  m_positionControl.FeedForward = feedforwardVolts;
  m_controller.setControl(m_positionControl);
}
public void setVelocitySetpoint(final int slot, final double setpoint) {
  setVelocitySetpoint(slot, setpoint, 0.0);
}

public void setVelocitySetpoint(
    final int slot, final double setpoint, final double feedforwardVolts) {
  setVelocitySetpoint(slot, setpoint, feedforwardVolts, false);
}

public void setVelocitySetpoint(
    final int slot,
    final double setpoint,
    final double feedforwardVolts,
    final boolean synchronous) {
  m_velocityControl.UpdateFreqHz = synchronous ? 0.0 : kDefaultUpdateFreqHz;
  m_velocityControl.Slot = slot;
  m_velocityControl.Velocity = toNativeSensorVelocity(setpoint);
  m_velocityControl.FeedForward = feedforwardVolts;
  m_controller.setControl(m_velocityControl);
}
public double getPercentOutput() {
  m_percentOutputSignal.refresh();
  return m_percentOutputSignal.getValue();
}

public double getPhysicalPercentOutput() {
  return (getInverted() ? -1.0 : 1.0) * getPercentOutput();
}

public double getSupplyCurrent() {
  return m_controller.getSupplyCurrent().getValue();
}

public double getStatorCurrent() {
  return m_controller.getStatorCurrent().getValue();
}

public boolean getInverted() {
  // This assumes that the config has been properly applied.
  return m_config.INVERTED;
}

public void zeroSensorPosition() {
  setSensorPosition(0.0);
}

public void setSensorPosition(final double pos) {
  // TODO: Handle zero offset internally.
  m_controller.setPosition(toNativeSensorPosition(pos));
}
public double getSensorPosition() {
  m_sensorPositionSignal.refresh();
  return m_sensorPositionSignal.getValue();
}

public double getSensorVelocity() {
  m_sensorVelocitySignal.refresh();
  return m_sensorVelocitySignal.getValue();
}

public MechanismRatio getMechanismRatio() {
  return m_ratio;
}
    
}
