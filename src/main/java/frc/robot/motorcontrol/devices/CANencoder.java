package frc.robot.motorcontrol.devices;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState; 
import frc.robot.motorcontrol.MechanismRatio;
import frc.robot.motorcontrol.configurations.phoenix.PhoenixUtils;
import frc.robot.motorcontrol.devices.EasyStatusSignal;

public class CANencoder {
    private static final double kCANTimeoutS = 0.1; // s
    private final CANcoder m_cancoder;
    //private final CANcoderSimState m_simState;
    private final MechanismRatio m_ratio;
    private final EasyStatusSignal m_positionSignal;
    private final EasyStatusSignal m_absolutePositionSignal;
    private final EasyStatusSignal m_velocitySignal;

    public CANencoder(final CANDeviceID canID, final MechanismRatio ratio) {
        m_cancoder = new CANcoder(canID.deviceNumber, canID.CANbusName);
        m_ratio = ratio;

        m_positionSignal = new EasyStatusSignal(m_cancoder.getPosition(), this::fromNativeSensorPosition);

        m_absolutePositionSignal = new EasyStatusSignal(m_cancoder.getAbsolutePosition(), this::fromNativeSensorPosition);

        m_velocitySignal = new EasyStatusSignal(m_cancoder.getVelocity(), this::fromNativeSensorVelocity);

        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.MagnetOffset = 0.0;
        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;

        PhoenixUtils.retryUntilSuccess(
        () -> m_cancoder.getConfigurator().apply(config, kCANTimeoutS),
        () -> {
          CANcoderConfiguration readConfig = new CANcoderConfiguration();
          m_cancoder.getConfigurator().refresh(readConfig, kCANTimeoutS);
          return PhoenixUtils.CANcoderConfigsEqual(config, readConfig);
        },
        "CANCoder " + canID + ": applyConfiguration");

        // Set update frequencies.
        PhoenixUtils.retryUntilSuccess(
            () -> m_positionSignal.setUpdateFrequency(10.0, kCANTimeoutS),
            () -> m_positionSignal.getAppliedUpdateFrequency() == 10.0,
            "CANCoder " + canID + ": m_positionSignal.setUpdateFrequency()");
        PhoenixUtils.retryUntilSuccess(
            () -> m_absolutePositionSignal.setUpdateFrequency(10.0, kCANTimeoutS),
            () -> m_absolutePositionSignal.getAppliedUpdateFrequency() == 10.0,
            "CANCoder " + canID + ": m_absolutePositionSignal.setUpdateFrequency()");
        PhoenixUtils.retryUntilSuccess(
            () -> m_velocitySignal.setUpdateFrequency(10.0, kCANTimeoutS),
            () -> m_velocitySignal.getAppliedUpdateFrequency() == 10.0,
            "CANCoder " + canID + ": m_velocitySignal.setUpdateFrequency()");

        // Disable all signals that have not been explicitly defined.
        PhoenixUtils.retryUntilSuccess(
            () -> m_cancoder.optimizeBusUtilization(kCANTimeoutS),
            "CANCoder " + canID + ": optimizeBusUtilization");

        // Block until we get valid signals.
        PhoenixUtils.retryUntilSuccess(
            () ->
                EasyStatusSignal.waitForAll(
                    kCANTimeoutS, m_positionSignal, m_absolutePositionSignal, m_velocitySignal),
            "CANCoder " + canID + ": waitForAll()");

        // Check if unlicensed.
        if (m_cancoder.getStickyFault_UnlicensedFeatureInUse().getValue()) {
        throw new RuntimeException("CANCoder " + canID + " is unlicensed!");

        }


    }

    // Expose status signals for timesync
    public EasyStatusSignal positionSignal() {
        return m_positionSignal;
    }

    public EasyStatusSignal absolutePositionSignal() {
        return m_absolutePositionSignal;
    }

    public EasyStatusSignal velocitySignal() {
        return m_velocitySignal;
    }

    public void zero() {
        setPosition(0.0);
    }

    public void setPosition(final double pos) {
        m_cancoder.setPosition(toNativeSensorPosition(pos));
    }

    public double getPosition() {
        m_positionSignal.refresh();
        return m_positionSignal.getValue();
      }
    
      public double getAbsPosition() {
        m_absolutePositionSignal.refresh();
        return m_absolutePositionSignal.getValue();
      }
    
      public double getVelocity() {
        m_velocitySignal.refresh();
        return m_velocitySignal.getValue();
      }
    
      private double toNativeSensorPosition(final double pos) {
        // Native units are in rotations.
        return m_ratio.mechanismPositionToSensorRadians(pos) / (2.0 * Math.PI);
      }
    
      private double fromNativeSensorPosition(final double pos) {
        return pos / toNativeSensorPosition(1.0);
      }
    
      private double toNativeSensorVelocity(final double vel) {
        return toNativeSensorPosition(vel);
      }
    
      private double fromNativeSensorVelocity(final double vel) {
        return vel / toNativeSensorVelocity(1.0);
      }

}
