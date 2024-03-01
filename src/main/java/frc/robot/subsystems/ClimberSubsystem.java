package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.motorcontrol.TalonFx;
import frc.robot.motorcontrol.configurations.TalonFxConfiguration;

public class ClimberSubsystem  extends SubsystemBase{
    
    private final TalonFx m_rightClimberMotor = new TalonFx(Constants.Climber.rightClimber.rightClimberID, 
    Constants.Climber.rightClimber.rightClimberRatio, 
    TalonFx.makeDefaultConfig().setInverted(Constants.Climber.rightClimber.rightClimberInvert)
    .setBrakeMode()
    .setPIDConfig(Constants.Climber.rightClimber.rightClimberMotorSlot, Constants.Climber.rightClimber.rightClimberPIDConfig)
    .setSupplyCurrentLimit(20.0)
    .setStatorCurrentLimit(20.0)
    .setReverseSoftLimit(Constants.Climber.minPosition)
    .setForwardSoftLimit(Constants.Climber.maxPosition)
    
    
    );
    //
     //     

    private final TalonFx m_leftClimberMotor = new TalonFx(Constants.Climber.leftClimber.leftClimberID, 
    Constants.Climber.leftClimber.leftClimberRatio, 
    TalonFx.makeDefaultConfig().setInverted(Constants.Climber.leftClimber.leftClimberInvert)
    .setBrakeMode()
    .setPIDConfig(Constants.Climber.leftClimber.leftClimberMotorSlot, Constants.Climber.leftClimber.leftClimberPIDConfig)
    .setSupplyCurrentLimit(20.0)
    .setStatorCurrentLimit(20.0)
    .setReverseSoftLimit(Constants.Climber.minPosition)
    .setForwardSoftLimit(Constants.Climber.maxPosition)
    
    
   
    );
   // .setReverseSoftLimit(Constants.Climber.minPosition)
  //  .setForwardSoftLimit(Constants.Climber.maxPosition)
  //  .setReverseSoftLimit(Constants.Climber.minPosition)
   // .setForwardSoftLimit(Constants.Climber.maxPosition)

   //  .setReverseSoftLimit(Constants.Climber.minPosition)
   // .setForwardSoftLimit(Constants.Climber.maxPosition)

    private final ElevatorFeedforward m_rightFF = Constants.Climber.rightClimber.rightFF;
    private final ElevatorFeedforward m_leftFF = Constants.Climber.leftClimber.leftFF;
   
    public ClimberSubsystem() {
      //Body
      //Show scheduler status in SmartDashboard.
      m_rightClimberMotor.zeroSensorPosition();
      m_leftClimberMotor.zeroSensorPosition();

      SmartDashboard.putData(this);

    }

    public void moveRightClimberUp() {
        double position = Constants.Climber.maxPosition;
        double rightFFVolts = m_rightFF.calculate(position);

        m_rightClimberMotor.setPositionSetpoint(Constants.Climber.rightClimber.rightClimberMotorSlot,rightFFVolts);

    }

    public void moveLeftClimberUp() {
        double position = Constants.Climber.maxPosition;
        
        double leftFFVolts = m_leftFF.calculate(position);
        m_leftClimberMotor.setPositionSetpoint(Constants.Climber.leftClimber.leftClimberMotorSlot,leftFFVolts);

    }

    public void moveRightClimberDown() { 
        double position = Constants.Climber.minPosition;
        double rightFFVolts = m_rightFF.calculate(position);

        m_rightClimberMotor.setPositionSetpoint(Constants.Climber.rightClimber.rightClimberMotorSlot, rightFFVolts);

    }

    public void moveLeftClimberDown() {
        double position = Constants.Climber.minPosition;
        
        double leftFFVolts = m_leftFF.calculate(position);
        m_leftClimberMotor.setPositionSetpoint(Constants.Climber.leftClimber.leftClimberMotorSlot,leftFFVolts);

    }

   /*public void moveClimber(double rightOut, double leftOut) {
        System.out.println("Right operator joystick output" + rightOut);
        System.out.println("Left operator joystick output" + leftOut);
        double rightVelo = rightOut * Constants.Climber.maxSpeed;
        double leftVelo = leftOut * Constants.Climber.maxSpeed;
        
        double rightFFVolts = m_rightFF.calculate(rightVelo);
        double leftFFVolts = m_leftFF.calculate(leftVelo);
        m_rightClimberMotor.setVelocitySetpoint(Constants.Climber.rightClimber.rightClimberMotorSlot,rightFFVolts);
        m_leftClimberMotor.setVelocitySetpoint(Constants.Climber.leftClimber.leftClimberMotorSlot,leftFFVolts);
    }*/

    public void moveClimber(double rightOut, double leftOut) {
       // System.out.println("Right operator joystick output" + rightOut);
        //System.out.println("Left operator joystick output" + leftOut);
        double pos = m_rightClimberMotor.getSensorPosition();
        System.out.println("Right Climber position" + pos);

        m_rightClimberMotor.setPercentOutput(rightOut);
        m_leftClimberMotor.setPercentOutput(leftOut);
    }

    

    public boolean isRightClimberPositionGood(double position) {
        if (position > Constants.Climber.maxPosition) {
            return false;
        }

        else {
            return true;
        }

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
    

