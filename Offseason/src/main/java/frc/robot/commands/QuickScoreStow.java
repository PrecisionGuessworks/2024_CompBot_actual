// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;


public class QuickScoreStow extends Command {
  private final ArmSubsystem m_arm;
  //private Pose2d m_pose;
  private Timer m_Timer = new Timer();
  private Boolean End = false;
  public QuickScoreStow(
      ArmSubsystem armSubsystem) {
    
    //m_pose = currentPose;
    m_arm = armSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_arm.isAtAngle(Constants.Arm.armShootAngle,Constants.Arm.AngleTolerance)
    &&m_arm.shooterAtSpeed(Constants.Arm.quickShootVelocity,Constants.Arm.ShootTolerance)) {

      m_arm.setAmpFeederVelocity(Constants.Arm.ampShootVelocity,Constants.Arm.feederShootVelocity);
      m_Timer.restart();
    } else {
      End = true;
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  m_arm.setArmAngle(Constants.Arm.armStowAngle);
  m_arm.setAmpFeederVelocity(0, 0);
  m_arm.setShooterVelocity(0);
  m_arm.Shoot = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Timer.get() > 0.35 || End;
  }
}
