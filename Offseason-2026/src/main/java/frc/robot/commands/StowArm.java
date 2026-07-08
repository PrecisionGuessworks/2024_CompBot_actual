// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class StowArm extends Command {
  private final ArmSubsystem m_arm;
 private final IntakeSubsystem m_intake;
  public StowArm(
      ArmSubsystem armSubsystem,
      IntakeSubsystem intakeSubsystem) {
    m_arm = armSubsystem;
    m_intake = intakeSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setArmAngle(Constants.Arm.armStowAngle);
    m_arm.setAmpFeederVelocity(0,0);
    m_arm.setShooterVelocity(0);
    m_intake.setRollerVelocity(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  // @Override
  // public void execute() {
  //   System.out.println(m_arm.getArmAngle());
  // }

  // Called once the command ends or is interrupted.
  // @Override
  // public void end(boolean interrupted) {
  //   m_arm.setArmAngle(Constants.Arm.armStowAngle);
  //   m_arm.setWristAngle(Constants.Arm.wristStowAngle);
  //   m_elevator.setHeight(Constants.Elevator.stowHeight);
  // }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
