package frc.robot.subsystems;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    private final TrapezoidProfile m_rightProfile = new TrapezoidProfile(Constants.Arm.RightPivot.rightPivotTrapConstraints);
    private final TrapezoidProfile m_leftProfile = new TrapezoidProfile(Constants.Arm.LeftPivot.leftPivotTrapConstraints);

    private final PositionVoltage m_rightMotorRequest = new PositionVoltage(0).withSlot(Constants.Arm.RightPivot.rightPivotMotorSlot);
    private final PositionVoltage m_leftMotorRequest = new PositionVoltage(0).withSlot(Constants.Arm.RightPivot.rightPivotMotorSlot);

    private double m_targetArmAngle = Constants.Arm.startingAngle;

    private TrapezoidProfile.State m_rightMotorSetpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_leftMotorSetpoint = new TrapezoidProfile.State();


    public ArmSubsystem() {
      //Body
      //Show scheduler status in SmartDashboard.
      m_armEncoder.setPosition(Constants.Arm.startingAngle);

      SmartDashboard.putData(this);

    }

    public double getArmAngle() {
      return m_armEncoder.getPosition();
    }

    public boolean isAtAngle(double angle, double tolerance) {
      return Math.abs(angle - getArmAngle()) <= tolerance;
    }

    public void setArmAngle(double targetArmAngle) {
     // m_rightMotorSetpoint = m_rightProfile.calculate()
    }

    @Override
    public void periodic() {
      m_rightMotorRequest.Position = m_rightMotorSetpoint.position;
      m_rightMotorRequest.Velocity = m_rightMotorSetpoint.velocity;
      m_rightMotor.m_controller.setControl(m_rightMotorRequest);
      // This method will be called once per scheduler run
      

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
    

