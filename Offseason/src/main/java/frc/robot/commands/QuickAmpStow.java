// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;


public class QuickAmpStow extends Command {
  private final ArmSubsystem m_arm;
  private Timer m_ScoreTimer = new Timer();
  private Timer m_Overide = new Timer();
  private Timer m_preScoreTimmer = new Timer();
  private Timer m_PostScoreTimer = new Timer();

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
    m_preScoreTimmer.restart();
    m_ScoreTimer.reset();
    m_PostScoreTimer.reset();
    if(!Robot.isReal()){
      double ShotVelocity = 100;
            Robot.updateNoteViz(new Pose3d(RobotContainer.drivetrain.getState().Pose.getX(),RobotContainer.drivetrain.getState().Pose.getY(),0.4, new Rotation3d(0,-Constants.Arm.armAmpAngle + Units.degreesToRadians(150),RobotContainer.drivetrain.getState().Pose.getRotation().getRadians())), 
            new double[] {RobotContainer.drivetrain.getState().Speeds.vxMetersPerSecond + ShotVelocity * Math.cos(Constants.Arm.armAmpAngle + Units.degreesToRadians(150))*Math.cos(RobotContainer.drivetrain.getState().Pose.getRotation().getRadians()),
              RobotContainer.drivetrain.getState().Speeds.vyMetersPerSecond + ShotVelocity * Math.cos(Constants.Arm.armAmpAngle + Units.degreesToRadians(150))*Math.sin(RobotContainer.drivetrain.getState().Pose.getRotation().getRadians()), 
              ShotVelocity * Math.sin(Constants.Arm.armAmpAngle + Units.degreesToRadians(150)) });
        }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_preScoreTimmer.get() < 0.2){
      m_arm.setAmpFeederVelocity(Constants.Arm.ampShootVelocity,Constants.Arm.feederShootVelocity);
      m_arm.setShooterVelocity(Constants.Arm.ampShootVelocity);
      m_ScoreTimer.start();
    }
    if (m_ScoreTimer.get() > 0.2){
      m_arm.setArmAngle(Constants.Arm.armPostAmpAngle);
      m_PostScoreTimer.start();
    }


    
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
    return m_PostScoreTimer.get() > 0.35 || End || m_Overide.get() > 1.5;
  }
}
