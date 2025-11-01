// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;


public class QuickScore extends Command {
  private final ArmSubsystem m_arm;
  //private Pose2d m_pose;

  public QuickScore(
      ArmSubsystem armSubsystem) {
    
    //m_pose = currentPose;
    m_arm = armSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setArmAngle(Constants.Arm.armShootAngle);
    m_arm.setShooterVelocity(Constants.Arm.quickShootVelocity);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_arm.isAtAngle(Constants.Arm.armShootAngle,Constants.Arm.AngleTolerance)
    &&m_arm.shooterAtSpeed(Constants.Arm.quickShootVelocity,Constants.Arm.ShootTolerance)){
      m_arm.Shoot = true;
    }
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  // m_arm.setArmAngle(Constants.Arm.armStowAngle);
  // m_arm.setShooterVelocity(0);
  // m_arm.setRollerVelocity(10);
  }

  // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return m_intake.hasPiece();
  // }
}
