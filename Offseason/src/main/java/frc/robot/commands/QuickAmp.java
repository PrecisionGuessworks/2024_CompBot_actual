// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.generated.Telemetry;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


public class QuickAmp extends Command {
  private final ArmSubsystem m_arm;
  //private Pose2d m_pose;

  public QuickAmp(
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
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  // m_arm.setArmAngle(Constants.Arm.armStowAngle);
  // m_arm.setShooterVelocity(0);
  m_arm.setRollerVelocity(10);
  }

  // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return m_intake.hasPiece();
  // }
}
