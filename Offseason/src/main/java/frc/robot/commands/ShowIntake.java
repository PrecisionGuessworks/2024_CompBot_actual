// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class Intake extends Command {
  private final ArmSubsystem m_arm;
private Timer m_placeTimer = new Timer();

  public Intake(
      ArmSubsystem armSubsystem) {
    m_arm = armSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(IntakeSubsystem, armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setArmAngle(Constants.Arm.armIntakeAngle);
    m_arm.setArmRollerCurrent(55, 90);  
    m_arm.setRollerVelocity(Constants.Arm.intakeVelocity);
    m_placeTimer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println(m_arm.getArmAngle());
    //m_elevator.setHeight(Constants.Elevator.stowHeight);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setArmAngle(Constants.Arm.armStowAngle);
    m_arm.setArmRollerCurrent(10, 10); 
    m_arm.setRollerVelocity(-20);
    RobotContainer.arm.setHasPiece(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_arm.isrollerStalled()&&m_placeTimer.hasElapsed(0.30);
  }
}
