// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CANdle;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANdleSubsystem extends SubsystemBase {
  
  private CANdle candle = new CANdle(28);

  

  /** Creates a new CANdleSubsystem. */
  public CANdleSubsystem() {
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB; // set the strip type to RGB
    config.brightnessScalar = 1; // dim the LEDs to half brightness
    candle.configAllSettings(config);
   
    candle.setLEDs(255, 255, 255); // set the CANdle LEDs to white
  }

  public void setCANdleToAllianceColor() {
    boolean isRed = NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(true);
    if (isRed == true){
        setColor(Colors.RED);
    } else {
        setColor(Colors.BLUE);
    }
}


  public void setColor(Colors color){
    candle.setLEDs(color.getRedVal(), color.getGreenVal(), color.getBlueVal());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
