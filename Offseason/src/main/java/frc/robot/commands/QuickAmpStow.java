// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;


public class QuickAmpStow extends Command {
  private final ArmSubsystem m_arm;
  private Timer m_ScoreTimer = new Timer();
  private Timer m_Overide = new Timer();
  private Timer m_preScoreTimmer = new Timer();

  private boolean End = false;
  //private Pose2d m_pose;

  public QuickAmpStow(
      ArmSubsystem armSubsystem) {
    
    //m_pose = currentPose;
    m_arm = armSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setArmAngle(Constants.Arm.armAmpAngle);
    m_Overide.restart();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setArmAngle(Constants.Arm.armStowAngle);
    m_arm.setShooterVelocity(0);
    m_arm.setAmpFeederVelocity(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_ScoreTimer.get() > 0.35 || End || m_Overide.get() > 1;
  }
}
