// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;


public class AimScore extends Command {
  private final ArmSubsystem m_arm;
  //private Pose2d m_pose;
  private double ShotVelocity = 0;


  public AimScore(
      ArmSubsystem armSubsystem) {
    
    m_arm = armSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setArmAngle(Constants.ShotCalc.Angle.get(RobotContainer.targetDistance()));
    m_arm.setShooterVelocity(Constants.ShotCalc.Velocity.get(RobotContainer.targetDistance()));
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.setArmAngle(Constants.ShotCalc.Angle.get(RobotContainer.targetDistance()));
    m_arm.setShooterVelocity(Constants.ShotCalc.Velocity.get(RobotContainer.targetDistance()));
    // System.out.println("Aiming to shoot at distance: " + RobotContainer.targetDistance() + " with angle: " + Constants.ShotCalc.Angle.get(RobotContainer.targetDistance()) + " and velocity: " + Constants.ShotCalc.Velocity.get(RobotContainer.targetDistance()));
    if (m_arm.isAtAngle(Constants.ShotCalc.Angle.get(RobotContainer.targetDistance()),Constants.Arm.AngleTolerance)
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
  if(!Robot.isReal()){
    ShotVelocity = Constants.ShotCalc.Velocity.get(RobotContainer.targetDistance()) * Constants.Arm.WheelRadius * Constants.Shotefficiency;  
        Robot.updateNoteViz(new Pose3d(RobotContainer.drivetrain.getState().Pose.getX(),RobotContainer.drivetrain.getState().Pose.getY(),0.4, new Rotation3d(0,-Constants.ShotCalc.Angle.get(RobotContainer.targetDistance()),RobotContainer.drivetrain.getState().Pose.getRotation().getRadians())), 
        new double[] {RobotContainer.drivetrain.getFieldSpeedsX() + ShotVelocity * Math.cos(Constants.ShotCalc.Angle.get(RobotContainer.targetDistance()))*Math.cos(RobotContainer.drivetrain.getState().Pose.getRotation().getRadians()),
          RobotContainer.drivetrain.getFieldSpeedsY() + ShotVelocity * Math.cos(Constants.ShotCalc.Angle.get(RobotContainer.targetDistance()))*Math.sin(RobotContainer.drivetrain.getState().Pose.getRotation().getRadians()), 
          ShotVelocity * Math.sin(Constants.ShotCalc.Angle.get(RobotContainer.targetDistance())) });
    }
  
}

  // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return m_intake.hasPiece();
  // }
}
