// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Blinkin;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BlinkinSubsystem extends SubsystemBase {
  /** Creates a new Blinkin. */
  private static BlinkinSubsystem instance;
  private Spark blinkin;
  private Colors color;
  
  public static BlinkinSubsystem getInstance(){
    if(instance == null) {
      instance = new BlinkinSubsystem();
    }
    return instance;
  }

  public BlinkinSubsystem() {
    blinkin = new Spark(Constants.Blinkin.BLINKIN_ID);
  }

  public void setColor(Colors chosenColor){
    blinkin.set(chosenColor.getColorVal());
  }
  
  public void setBlinkinToAllianceColor() {
      boolean isRed = NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(true);
      if (isRed == true){
          setColor(Colors.RED);
      } else {
          setColor(Colors.BLUE);
      }
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
