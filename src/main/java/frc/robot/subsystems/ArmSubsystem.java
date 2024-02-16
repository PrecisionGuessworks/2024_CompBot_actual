package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.motorcontrol.TalonFx;
import frc.robot.motorcontrol.configurations.TalonFxConfiguration;
import frc.robot.motorcontrol.devices.CANencoder;

public class ArmSubsystem  extends SubsystemBase{

  private final TalonFx m_rightMotor =
  new TalonFx(
      Constants.Arm.RightPivot.rightPivotID,
      Constants.Arm.RightPivot.rightPivotRatio,
      TalonFx.makeDefaultConfig()
          .setInverted(Constants.Arm.RightPivot.rightPivotInvert)
          .setBrakeMode()
          .setPIDConfig(Constants.Arm.RightPivot.rightPivotMotorSlot, Constants.Arm.RightPivot.rightPivotPIDConfig)
          .setSupplyCurrentLimit(40.0)
          .setStatorCurrentLimit(40.0)
    );

    private final TalonFx m_leftMotor =
  new TalonFx(
      Constants.Arm.LeftPivot.leftPivotID,
      Constants.Arm.LeftPivot.leftPivotRatio,
      TalonFx.makeDefaultConfig()
          .setInverted(Constants.Arm.LeftPivot.leftPivotInvert)
          .setBrakeMode()
          .setPIDConfig(Constants.Arm.LeftPivot.leftPivotMotorSlot, Constants.Arm.LeftPivot.leftPivotPIDConfig)
          .setSupplyCurrentLimit(40.0)
          .setStatorCurrentLimit(40.0)
    );

    private final CANencoder m_armEncoder  = new CANencoder(Constants.Arm.ArmEnconder.encoderID, Constants.Arm.ArmEnconder.armRatio);
    

    private double m_targetArmAngle = Constants.Arm.startingAngle;

    public ArmSubsystem() {
      //Body
      //Show scheduler status in SmartDashboard.
      m_armEncoder.setPosition(Constants.Arm.startingAngle);
      m_rightMotor.setSensorPosition(Constants.Arm.startingAngle);
      m_leftMotor.setSensorPosition(Constants.Arm.startingAngle);

      SmartDashboard.putData(this);

    }

    public double getArmAngle() {
      return m_armEncoder.getPosition();
    }

    public boolean isAtAngle(double angle, double tolerance) {
      return Math.abs(angle - getArmAngle()) <= tolerance;
    }

    public void setArmAngle(double targetArmAngle) {
      m_targetArmAngle = targetArmAngle;
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run

      m_rightMotor.setPositionSetpoint(Constants.Arm.RightPivot.rightPivotMotorSlot, m_targetArmAngle);
      m_leftMotor.setPositionSetpoint(Constants.Arm.LeftPivot.leftPivotMotorSlot, m_targetArmAngle);

      SmartDashboard.putNumber(
        "Launcher: Current Arm Angle (deg)",
        Units.radiansToDegrees(m_armEncoder.getPosition()));
      SmartDashboard.putNumber(
        "Launcher: Target Arm Angle (deg)", Units.radiansToDegrees(m_targetArmAngle));
    }
  
    // --- BEGIN STUFF FOR SIMULATION ---
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
    // --- END STUFF FOR SIMULATION ---
  }
    

