// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.quixlib.motorcontrol.QuixTalonFX;
import frc.quixlib.viz.Link2d;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  public final DigitalInput m_beamBreak = new DigitalInput(Constants.Intake.beamBreakPort);
  

  private final QuixTalonFX m_rollerMotor =
      new QuixTalonFX(
          Constants.Intake.rollerMotorID,
          Constants.Intake.rollerMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Intake.rollerMotorInvert)
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(80.0)
              .setBrakeMode()
              .setPIDConfig(Constants.Intake.rollerVelocitySlot, Constants.Intake.rollerPIDConfig));



  public IntakeSubsystem(Link2d intakeRollerViz) {

    SmartDashboard.putData(this);


    m_intakeRollerViz = intakeRollerViz;
  }


  public void setRollerVelocity(double velocity) {
    if (velocity == 0.0) {
      m_rollerMotor.setPercentOutput(0.0);
    } else {
      m_rollerMotor.setVelocitySetpoint(
          Constants.Intake.rollerVelocitySlot,
          velocity,
          Constants.Intake.rollerFeedforward.calculate(velocity));
    }
  }
  
  public boolean hasPiece() {
    return !m_beamBreak.get();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run


    SmartDashboard.putBoolean("Intake: Beam Break", m_beamBreak.get());
    SmartDashboard.putBoolean("Intake: Has Piece", hasPiece());

    m_rollerMotor.logMotorState();
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  static final DCMotor m_simMotor = DCMotor.getKrakenX60Foc(1);
  private static final FlywheelSim m_rollerSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              m_simMotor,
              Constants.Intake.simRollerMOI,
              Constants.Intake.rollerMotorRatio.reduction()),
          m_simMotor);

          
  // Visualization
  private final Link2d m_intakeRollerViz;

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    
    m_rollerSim.setInput(m_rollerMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_rollerSim.update(TimedRobot.kDefaultPeriod);
    m_rollerMotor.setSimSensorVelocity(
        m_rollerSim.getAngularVelocityRadPerSec(),
        TimedRobot.kDefaultPeriod,
        Constants.Intake.deployMotorRatio);

    m_intakeRollerViz.setRelativeTransform(
        new Transform2d(
            Constants.Viz.intakeArmLength,
            0.0,
            Rotation2d.fromRadians(
                m_intakeRollerViz.getRelativeTransform().getRotation().getRadians()
                    + m_rollerSim.getAngularVelocityRadPerSec()
                        * Constants.Viz.angularVelocityScalar)));
  }
  // --- END STUFF FOR SIMULATION ---
}
