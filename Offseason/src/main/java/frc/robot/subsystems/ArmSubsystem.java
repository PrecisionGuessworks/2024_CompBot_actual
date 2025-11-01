// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.Serial;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.ImmutableAngle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.quixlib.devices.QuixCANCoder;
import frc.quixlib.motorcontrol.QuixTalonFX;
import frc.quixlib.viz.Link2d;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArmSubsystem extends SubsystemBase {
  //public final DigitalInput m_beamBreak = new DigitalInput(Constants.Arm.beamBreakPort);

  static private final QuixCANCoder m_armCoder = 
      new QuixCANCoder(Constants.Arm.armCoderID, Constants.Arm.armMotorRatio, SensorDirectionValue.Clockwise_Positive);
  
      static double ArmStartingAngle = Constants.Arm.armStartingAngle;
     // : Units.rotationsToRadians(m_armCoder.getAbsPosition()); Constants.isSim ? 
  private final QuixTalonFX m_ampMotor =
      new QuixTalonFX(
          Constants.Arm.ampMotorID,
          Constants.Arm.ampMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Arm.ampMotorInvert)
              .setSupplyCurrentLimit(30.0)
              .setStatorCurrentLimit(50.0)
              .setPIDConfig(Constants.Arm.ampVelocityPIDSlot, Constants.Arm.ampPositionPIDConfig));

  private final QuixTalonFX m_feederMotor =
      new QuixTalonFX(
          Constants.Arm.feederMotorID,
          Constants.Arm.feederMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Arm.feederMotorInvert)
              .setSupplyCurrentLimit(30.0)
              .setStatorCurrentLimit(50.0)
              .setPIDConfig(Constants.Arm.feederVelocityPIDSlot, Constants.Arm.feederPositionPIDConfig));

  private final QuixTalonFX m_shooterUpperMotor =
      new QuixTalonFX(
          Constants.Arm.shooterUpperMotorID,
          Constants.Arm.shooterUpperMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Arm.shooterUpperMotorInvert)
              .setSupplyCurrentLimit(50.0)
              .setStatorCurrentLimit(120.0)
              .setPIDConfig(Constants.Arm.shooterUpperVelocityPIDSlot, Constants.Arm.shooterUpperPositionPIDConfig));

    private final QuixTalonFX m_shooterLowerMotor =
      new QuixTalonFX(
          Constants.Arm.shooterLowerMotorID,
          Constants.Arm.shooterLowerMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Arm.shooterLowerMotorInvert)
              .setSupplyCurrentLimit(50.0)
              .setStatorCurrentLimit(120.0)
              .setPIDConfig(Constants.Arm.shooterLowerVelocityPIDSlot, Constants.Arm.shooterLowerPositionPIDConfig));

  private final QuixTalonFX m_armMotor =
      new QuixTalonFX(
          Constants.Arm.armMotorID,
          Constants.Arm.armMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Arm.armMotorInvert)
              .setBrakeMode()
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(80.0)
              .setMotionMagicConfig(
                  Constants.Arm.ArmConstraints.maxVelocity,
                  Constants.Arm.ArmConstraints.maxAcceleration,
                  Constants.Arm.ArmMaxJerk,
                  Constants.Arm.armExpo_kV,
                  Constants.Arm.armExpo_kA)

              .setPIDConfig(Constants.Arm.armPositionPIDSlot, Constants.Arm.armPositionPIDConfig)
              .setBootPositionOffset(ArmStartingAngle)
              .setReverseSoftLimit(Constants.Arm.armMinAngle)
              .setForwardSoftLimit(Constants.Arm.armMaxAngle)
            //  .setFeedbackConfig(FeedbackSensorSourceValue.FusedCANcoder, 15, 0.0,Constants.Arm.armMotorRatio,Constants.Arm.armSensorRatio)
              );

    private final QuixTalonFX m_armfollower = new QuixTalonFX(
      Constants.Arm.armfollowerID,
      m_armMotor,
      Constants.Arm.armfollowerInvert,
      QuixTalonFX.makeDefaultConfig()
      .setBrakeMode()
      .setSupplyCurrentLimit(40.0)
      .setStatorCurrentLimit(80.0)
      .setInverted(Constants.Arm.armfollowerInvert)
      .setPIDConfig(Constants.Arm.armPositionPIDSlot, Constants.Arm.armPositionPIDConfig)
      .setBootPositionOffset(ArmStartingAngle)
      .setMotionMagicConfig(
                  Constants.Arm.ArmConstraints.maxVelocity,
                  Constants.Arm.ArmConstraints.maxAcceleration,
                  Constants.Arm.ArmMaxJerk,
                  Constants.Arm.armExpo_kV,
                  Constants.Arm.armExpo_kA)
      .setReverseSoftLimit(Constants.Arm.armMinAngle)
      .setForwardSoftLimit(Constants.Arm.armMaxAngle));

  private double m_armTargetAngle = ArmStartingAngle;
  private double setm_armTargetAngle = ArmStartingAngle;
  private boolean hasPiece = true;
  private Timer m_lastPieceTimer = new Timer();

  public ArmSubsystem(Link2d ArmViz, Link2d ampViz, Link2d feederViz, Link2d shooterUpperViz, Link2d shooterLowerViz) {
    m_lastPieceTimer.start();
    m_lastPieceTimer.reset();

    // Show scheduler status in SmartDashboard.
    SmartDashboard.putData(this);

    // Setup viz.
    m_ArmArmViz = ArmViz;
    m_ArmampViz = ampViz;
    m_ArmfeederViz = feederViz;
    m_ArmshooterUpperViz = shooterUpperViz;
    m_ArmshooterLowerViz = shooterLowerViz;
  
  }

  // public boolean hasPiece() {
  //   return m_beamBreak.get();
  // }

  public boolean recentlyHadPiece() {
    return m_lastPieceTimer.get() < 1.0;
    }

    public double getArmAngle() { 
    return Units.radiansToDegrees(m_armMotor.getSensorPosition()) * Constants.Arm.armMotorRatio.inverseReduction() + Units.radiansToDegrees(ArmStartingAngle) ;
    //: Units.rotationsToRadians(m_armCoder.getAbsPosition());   Constants.isSim ? 
    }
    
  public double getampCurrent() {
    return m_ampMotor.getSupplyCurrent();
  }

  public double getArmCoder(){
    return Units.rotationsToDegrees(m_armCoder.getAbsPosition());
  }

  public void setArmAngle(double targetAngle) {
    setm_armTargetAngle = targetAngle;
  }
  

  public void setHasPiece(boolean thasPiece) {
    hasPiece = thasPiece;
  }
  public boolean getHasPiece() {
    return hasPiece;
  }


  public void setArmampCurrent(double StatorCurrentLimit, double SupplyCurrentLimit) {
    m_ampMotor.setStatorCurrentLimit(StatorCurrentLimit,SupplyCurrentLimit);
  }

  public boolean isAtAngle(double angle, double tolerance) {
    return Math.abs(angle - m_armMotor.getSensorPosition()) <= tolerance;
  }

  public boolean isrollerStalled() {
    //return Math.abs(m_rollerMotor.getSensorVelocity()) < Constants.Arm.rollerStallVelocity;
    //return m_rollerMotor.getSupplyCurrent() > Constants.Arm.rollerStallCurrent;
  return false;
  }
  public void setShooterVelocity(double Velocity) {
    if (Velocity == 0.0) {
      m_shooterUpperMotor.setPercentOutput(0.0);
      m_shooterLowerMotor.setPercentOutput(0.0);
    } else {
      m_shooterUpperMotor.setVelocitySetpoint(
          Constants.Arm.shooterUpperVelocityPIDSlot,
          Velocity,
          Constants.Arm.shooterUpperFeedforward.calculate(Velocity));
      m_shooterLowerMotor.setVelocitySetpoint(
          Constants.Arm.shooterLowerVelocityPIDSlot,
          -Velocity,
          Constants.Arm.shooterLowerFeedforward.calculate(-Velocity));
    }
  }

  public void setAmpFeederVelocity(double Amp, double Feeder) {
    if (Amp == 0.0) {
      m_ampMotor.setPercentOutput(0.0);
    } else {
      m_ampMotor.setVelocitySetpoint(
          Constants.Arm.ampVelocityPIDSlot,
          Amp,
          Constants.Arm.ampFeedforward.calculate(Amp));
    }
    if (Feeder == 0.0) {
      m_feederMotor.setPercentOutput(0.0);
    } else {
      m_feederMotor.setVelocitySetpoint(
          Constants.Arm.feederVelocityPIDSlot,
          Feeder,
          Constants.Arm.feederFeedforward.calculate(Feeder));
    }
  }

  // public void setRollerVelocityandCurrent(double velocity,double StatorCurrentLimit, double SupplyCurrentLimit) {
  //   m_rollerMotor.setStatorCurrentLimit(StatorCurrentLimit,SupplyCurrentLimit);
  //   if (velocity == 0.0) {
  //     m_rollerMotor.setPercentOutput(0.0);
  //   } else {
  //     m_rollerMotor.setVelocitySetpoint(
  //         Constants.Arm.rollerVelocityPIDSlot,
  //         velocity,
  //         Constants.Arm.rollerFeedforward.calculate(velocity));
  //   }
  // }

  // public void disabledInit() {
  //   m_armMotor.setBrakeMode(true);
  // }

  // public void disabledExit() {
  //   m_armMotor.setBrakeMode(false);
  // }

  //private double elevatorLocation = 0; 

  @Override
  public void periodic() {


    if (setm_armTargetAngle <= Constants.Arm.armMaxAngle && setm_armTargetAngle >= Constants.Arm.armMinAngle){ 
      m_armTargetAngle = setm_armTargetAngle;
    }else {
      m_armTargetAngle = Constants.Arm.armStowAngle;
    }
    

    // if(hasPiece){
    //   m_armMotor.setMotionMagicPositionSetpoint(
    //     Constants.Arm.armCoralPositionPIDSlot, m_armTargetAngle);
    // } else {
      // m_armMotor.setMotionMagicPositionSetpoint(
      //   Constants.Arm.armPositionPIDSlot, m_armTargetAngle);
      m_armMotor.setMotionMagicPositionSetpointExpo(
          Constants.Arm.armPositionPIDSlot, m_armTargetAngle);

 
   // }
  if(Constants.ExtraInfo){
    SmartDashboard.putNumber(
        "Arm: Current Angle (deg)", Units.radiansToDegrees(m_armMotor.getSensorPosition()));
        SmartDashboard.putNumber(
        "Arm: Current CANcoder Angle (deg)", getArmCoder());
    SmartDashboard.putNumber(
        "Arm: Real Current Angle (deg)", getArmAngle());
    SmartDashboard.putNumber(
        "Arm: Target Angle (deg)",
        Units.radiansToDegrees(m_armMotor.getClosedLoopReference()));
    SmartDashboard.putNumber(
        "Arm: Target set Angle (deg)",
        Units.radiansToDegrees(m_armTargetAngle));
    SmartDashboard.putNumber(
        "Arm: Current Velocity (deg per sec)",
        Units.radiansToDegrees(m_armMotor.getSensorVelocity()));
    SmartDashboard.putNumber(
        "Arm: Target Velocity (deg per sec)",
        Units.radiansToDegrees(m_armMotor.getClosedLoopReferenceSlope()));
    SmartDashboard.putNumber(
        "Arm: Current amp Velocity (rad per sec)", m_ampMotor.getSensorVelocity());

  }

  if(Constants.ExtraInfo){
    m_ampMotor.logMotorState();
    m_armMotor.logMotorState();
    m_armCoder.logSensorState();
  }
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  
  private static final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500Foc(2),
          Constants.Arm.armMotorRatio.reduction(),
          Constants.Arm.simArmMOI,
          Constants.Arm.simArmCGLength,
          Constants.Arm.armMinAngle,
          Constants.Arm.armMaxAngle,
          true, // Simulate gravity
          ArmStartingAngle);

  static final DCMotor m_simMotor = DCMotor.getFalcon500Foc(1);
  private static final FlywheelSim m_ampSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              m_simMotor,
              Constants.Arm.simRollerMOI,
              Constants.Arm.ampMotorRatio.reduction()),
          m_simMotor);

          
  // Visualization
  private final Link2d m_ArmArmViz;
  private final Link2d m_ArmampViz;
  private final Link2d m_ArmfeederViz;
  private final Link2d m_ArmshooterUpperViz;
  private final Link2d m_ArmshooterLowerViz;

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_armSim.setInput(m_armMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_armSim.update(TimedRobot.kDefaultPeriod);
    m_armMotor.setSimSensorPositionAndVelocity(
        m_armSim.getAngleRads() - ArmStartingAngle,
        // m_armSim.getVelocityRadPerSec(), // TODO: Figure out why this causes jitter
        0.0,
        TimedRobot.kDefaultPeriod,
        Constants.Arm.armMotorRatio);
    

    m_ampSim.setInput(m_ampMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_ampSim.update(TimedRobot.kDefaultPeriod);
    m_ampMotor.setSimSensorVelocity(
        m_ampSim.getAngularVelocityRadPerSec(),
        TimedRobot.kDefaultPeriod,
        Constants.Arm.armMotorRatio);

    // Update arm viz.
    m_ArmArmViz.setRelativeTransform(
        new Transform2d(
            Constants.Viz.ArmArmPivotX,
            0,
            Rotation2d.fromRadians(m_armSim.getAngleRads() + Units.degreesToRadians(- Constants.Viz.elevatorAngle.getDegrees()))));


    m_ArmampViz.setRelativeTransform(
        new Transform2d(
            Constants.Viz.ArmWristLength,
            0.0,
            Rotation2d.fromRadians(
                m_ArmampViz.getRelativeTransform().getRotation().getRadians()
                    + m_ampSim.getAngularVelocityRadPerSec()
                        * Constants.Viz.angularVelocityScalar)));
  }
  // --- END STUFF FOR SIMULATION ---
}