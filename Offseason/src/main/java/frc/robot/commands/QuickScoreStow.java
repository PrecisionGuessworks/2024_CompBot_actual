// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;


public class QuickScoreStow extends Command {
  private final ArmSubsystem m_arm;
  //private Pose2d m_pose;
  private Timer m_Timer = new Timer();
  private Boolean End = false;
  private double ShotVelocity = 0;
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
    // if (m_arm.isAtAngle(Constants.Arm.armShootAngle,Constants.Arm.AngleTolerance)
    // &&m_arm.shooterAtSpeed(Constants.Arm.quickShootVelocity,Constants.Arm.ShootTolerance)) {

      m_arm.setAmpFeederVelocity(Constants.Arm.ampShootVelocity,Constants.Arm.feederShootVelocity);
      m_Timer.restart();
      if(!Robot.isReal()){
        ShotVelocity = Constants.Arm.quickShootVelocity * Constants.Arm.WheelRadius * Constants.Shotefficiency;  
            Robot.updateNoteViz(new Pose3d(RobotContainer.drivetrain.getState().Pose.getX(),RobotContainer.drivetrain.getState().Pose.getY(),0.4, new Rotation3d(0,-Constants.Arm.armShootAngle,RobotContainer.drivetrain.getState().Pose.getRotation().getRadians())), 
            new double[] {RobotContainer.drivetrain.getFieldSpeedsX() + ShotVelocity * Math.cos(Constants.Arm.armShootAngle)*Math.cos(RobotContainer.drivetrain.getState().Pose.getRotation().getRadians()),
              RobotContainer.drivetrain.getFieldSpeedsY() + ShotVelocity * Math.cos(Constants.Arm.armShootAngle)*Math.sin(RobotContainer.drivetrain.getState().Pose.getRotation().getRadians()), 
              ShotVelocity * Math.sin(Constants.Arm.armShootAngle) });
        }
    // } else {
    //   End = true;
    // }
    
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
    return m_Timer.get() > 0.22 || End;
  }
}
