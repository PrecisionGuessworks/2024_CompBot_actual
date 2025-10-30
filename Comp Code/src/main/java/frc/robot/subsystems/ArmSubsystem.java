package frc.robot.subsystems;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.motorcontrol.TalonFx;
import frc.robot.motorcontrol.configurations.TalonFxConfiguration;
import frc.robot.motorcontrol.devices.CANencoder;

public class ArmSubsystem  extends SubsystemBase{

  private final CANencoder m_armEncoder  = new CANencoder(Constants.Arm.ArmEnconder.encoderID, Constants.Arm.ArmEnconder.armRatio);

  private final TalonFx m_rightMotor =
  new TalonFx(
      Constants.Arm.RightPivot.rightPivotID,
      Constants.Arm.RightPivot.rightPivotRatio,
      TalonFx.makeDefaultConfig()
          .setInverted(Constants.Arm.RightPivot.rightPivotInvert)
          .setBrakeMode()
          .setPIDConfig(Constants.Arm.RightPivot.rightPivotMotorSlot, Constants.Arm.RightPivot.rightPivotPIDConfig)
          .setSupplyCurrentLimit(45)
          .setStatorCurrentLimit(50)
          .setReverseSoftLimit(Constants.Arm.minAngle)
          .setForwardSoftLimit(Constants.Arm.maxAngle)
    );
    //.setFusedCANCoder(m_armEncoder, Constants.Arm.ArmEnconder.armRatio, Constants.Arm.ArmEnconder.encoderToMotorRatio)

   /* private final TalonFx m_leftMotor =
  new TalonFx(
      Constants.Arm.LeftPivot.leftPivotID,
      Constants.Arm.LeftPivot.leftPivotRatio,
      TalonFx.makeDefaultConfig()
          .setInverted(Constants.Arm.LeftPivot.leftPivotInvert)
          .setBrakeMode()
          .setPIDConfig(Constants.Arm.LeftPivot.leftPivotMotorSlot, Constants.Arm.LeftPivot.leftPivotPIDConfig)
          .setSupplyCurrentLimit(35)
          .setStatorCurrentLimit(35)
          .setReverseSoftLimit(Constants.Arm.minAngle)
          .setForwardSoftLimit(Constants.Arm.maxAngle)
    ); */
    

    private final TalonFx m_leftMotor = new TalonFx(Constants.Arm.LeftPivot.leftPivotID, 
    m_rightMotor, 
    Constants.Arm.LeftPivot.leftPivotInvert,
    TalonFx.makeDefaultConfig()
          .setInverted(Constants.Arm.LeftPivot.leftPivotInvert)
          .setBrakeMode()
          .setPIDConfig(Constants.Arm.LeftPivot.leftPivotMotorSlot, Constants.Arm.LeftPivot.leftPivotPIDConfig)
          .setSupplyCurrentLimit(45)
          .setStatorCurrentLimit(50)
          .setReverseSoftLimit(Constants.Arm.minAngle)
          .setForwardSoftLimit(Constants.Arm.maxAngle)
    );
     
    //.setFusedCANCoder(m_armEncoder, Constants.Arm.ArmEnconder.armRatio, Constants.Arm.ArmEnconder.encoderToMotorRatio)

    
    private final TrapezoidProfile m_rightProfile = new TrapezoidProfile(Constants.Arm.RightPivot.rightPivotTrapConstraints);
    //private final TrapezoidProfile m_leftProfile = new TrapezoidProfile(Constants.Arm.LeftPivot.leftPivotTrapConstraints);
    //private final TrapezoidProfile m_rightProfileDown = new TrapezoidProfile(Constants.Arm.RightPivot.rightPivotTrapConstraintsDown);
    //private final TrapezoidProfile m_leftProfileDown = new TrapezoidProfile(Constants.Arm.LeftPivot.leftPivotTrapConstraintsDown);

   
    private double m_targetArmAngle = Constants.Arm.startingAngle;

    

    private TrapezoidProfile.State m_rightArmState = new State(m_rightMotor.getSensorPosition(), 0.0);
    //private TrapezoidProfile.State m_leftArmState = new State(m_leftMotor.getSensorPosition(), 0.0);

    private final Timer m_armTrapTimer = new Timer();


    public ArmSubsystem() {
      m_rightMotor.zeroSensorPosition();
      //m_leftMotor.zeroSensorPosition();
      m_armTrapTimer.start();
      //Body
      //Show scheduler status in SmartDashboard.
      //m_armEncoder.setPosition(Constants.Arm.startingAngle);

      

      SmartDashboard.putData(this);

    }
    //Absolute encoder position
    public double getArmAngle() {
      return m_rightMotor.getSensorPosition();
    }

    public boolean isAtAngle(double angle, double tolerance) {
      return Math.abs(angle - getArmAngle()) <= tolerance;
    }

    public void setArmAngle(double targetArmAngle) {
      TrapezoidProfile.State m_goal = new TrapezoidProfile.State(targetArmAngle, 0);
      m_targetArmAngle = targetArmAngle;
      
      m_rightArmState = m_rightProfile.calculate(m_armTrapTimer.get(), m_rightArmState, m_goal);
      //m_leftArmState = m_leftProfile.calculate(m_armTrapTimer.get(),  m_leftArmState, m_goal);
      m_armTrapTimer.reset();
    }

    /*public void resetEncoders(double pos) {
      m_rightMotor.setSensorPosition(pos);
      m_leftMotor.setSensorPosition(pos);
    }*/

    /*public void setArmAngleDown(double targetArmAngle) {
      TrapezoidProfile.State m_goal = new TrapezoidProfile.State(targetArmAngle, 0);
      m_targetArmAngle = targetArmAngle;
      
      m_rightArmState = m_rightProfileDown.calculate(m_armTrapTimer.get(), m_rightArmState, m_goal);
      m_leftArmState = m_leftProfileDown.calculate(m_armTrapTimer.get(),  m_leftArmState, m_goal);
      m_armTrapTimer.reset();
    }*/

    public boolean isArmMotionFinished() {
      return ((m_armTrapTimer.get() > m_rightProfile.totalTime()));
    }

    @Override
    public void periodic() {
      if (DriverStation.isDisabled()) {
      // Update state to sensor state when disabled to prevent jumps on enable.
      m_rightArmState = new State(getArmAngle(), 0.0);
      //m_leftArmState = new State(getArmAngle(), 0.0);
    //  System.out.println("Launcher: Current Arm Angle (deg) " + m_armEncoder.getAbsPosition());
    //  System.out.println("Launcher: Target Arm Angle (deg) " + m_targetArmAngle);
    }
    
    
    m_rightMotor.setPositionSetpoint(Constants.Arm.RightPivot.rightPivotMotorSlot, m_rightArmState.position, Constants.Arm.RightPivot.rightFeedForward.calculate(
      m_rightArmState.position, m_rightArmState.velocity));

    //m_leftMotor.setPositionSetpoint(Constants.Arm.LeftPivot.leftPivotMotorSlot, m_leftArmState.position, Constants.Arm.LeftPivot.leftFeedForward.calculate(
      //m_leftArmState.position, m_leftArmState.velocity));

      /*m_rightMotorRequest.Position = m_rightMotorSetpoint.position;
      m_rightMotorRequest.Velocity = m_rightMotorSetpoint.velocity;
      m_leftMotorRequest.Position = m_leftMotorSetpoint.position;
      m_leftMotorRequest.Velocity = m_leftMotorSetpoint.velocity;

      m_rightMotor.m_controller.setControl(m_rightMotorRequest);
      m_leftMotor.m_controller.setControl(m_leftMotorRequest); */


      // This method will be called once per scheduler run
      

      //SmartDashboard.putNumber( "Launcher: Current Arm Angle (deg)", Units.radiansToDegrees(m_armEncoder.getAbsPosition()));
     // SmartDashboard.putNumber("Launcher: Target Arm Angle (deg)", Units.radiansToDegrees(m_targetArmAngle));

       // System.out.println("Launcher: Current Arm Angle  " + m_armEncoder.getAbsPosition());
       // System.out.println("Launcher: Target Arm Angle  " + m_targetArmAngle);
    }
  
    // --- BEGIN STUFF FOR SIMULATION ---
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
    // --- END STUFF FOR SIMULATION ---
  }
    

