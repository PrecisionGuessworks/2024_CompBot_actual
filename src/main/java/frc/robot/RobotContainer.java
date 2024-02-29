// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.AutoAim;
import frc.robot.commands.EjectPiece;
import frc.robot.commands.IntakePiece;
import frc.robot.commands.MoveArmAmp;
import frc.robot.commands.MoveArmSpeaker;
import frc.robot.commands.MoveClimber;
import frc.robot.commands.ScoreAmp;
import frc.robot.commands.MoveArmIntake;
import frc.robot.commands.ShootNoteSpeaker;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PresPoseEstimator;
import frc.robot.subsystems.ShooterSubsystem;


public class RobotContainer {
  final double MaxSpeed = 5.0292; // 6 meters per second desired top speed
  final double MaxAngularRate = 2 * Math.PI; // Half a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  //CommandPS4Controller joystick = new CommandPS4Controller(0);
  public final XboxController joystick = new XboxController(0); // My joystick
  public final XboxController operator = new XboxController(1); //operator

  CommandSwerveDrivetrain drivetrain = Constants.Swerve.TunerConstants.DriveTrain; // My drivetrain
  SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                                                            // driving in open loop
  SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  Telemetry logger = new Telemetry(MaxSpeed);

  PhotonCamera aprilCam = new PhotonCamera("OV2311");
  Transform3d robotToCam = new Transform3d(new Translation3d(0.0, 0.44, 0.37), new Rotation3d(0,Units.degreesToRadians(15),0));

  //Subsystems
  IntakeSubsystem intake = new IntakeSubsystem();
  ShooterSubsystem shooter = new ShooterSubsystem();
  ArmSubsystem arm = new ArmSubsystem();
  ClimberSubsystem climber = new ClimberSubsystem();
  //PresPoseEstimator poseEstimator = new PresPoseEstimator(aprilCam, drivetrain, robotToCam);

  Map<String, Command> robotCommands  = new HashMap<String, Command>();

  

  //NamedCommands.registerCommand();

  private Command runAuto = drivetrain.getAutoPath("Top Front");

  
  
  private final Trigger rightTrigger = new Trigger(() -> joystick.getRightTriggerAxis() > 0.2);
  private final Trigger leftTrigger = new Trigger(() -> joystick.getLeftTriggerAxis() > 0.2);
  private final Trigger operatorLeftTrigger = new Trigger(() -> operator.getLeftTriggerAxis() > 0.2);

  private final JoystickButton buttonA =
      new JoystickButton(joystick, XboxController.Button.kA.value);
  private final JoystickButton buttonB =
      new JoystickButton(joystick, XboxController.Button.kB.value);
  private final JoystickButton buttonX =
      new JoystickButton(joystick, XboxController.Button.kX.value);
  private final JoystickButton buttonY =
      new JoystickButton(joystick, XboxController.Button.kY.value);

  private final JoystickButton bumperLeft =
      new JoystickButton(joystick, XboxController.Button.kLeftBumper.value);
  private final JoystickButton bumperRight =
      new JoystickButton(joystick, XboxController.Button.kRightBumper.value);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    buttonY.whileTrue(drivetrain.applyRequest(() -> brake));
    buttonA.whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    

    // reset the field-centric heading on left bumper press
    bumperLeft.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    //shoot da note
    leftTrigger.whileTrue(new EjectPiece(shooter, arm));

    bumperRight.whileTrue(new ShootNoteSpeaker(shooter, arm));
    //bumperRight.onFalse(new MoveArmIntake(arm));

    //intake piece
    rightTrigger.whileTrue(new IntakePiece(intake, shooter, arm));

    //move arm
    buttonX.whileTrue(new ScoreAmp(shooter, arm));

    //buttonB.whileTrue(drivetrain.followTrajectoryCommand());

    operatorLeftTrigger.whileTrue(new MoveClimber(climber, operator));

    //climber.setDefaultCommand(new MoveClimber(climber, operator.getRightY(), operator.getLeftY()));
    

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    robotCommands.put("IntakePiece", new IntakePiece(intake, shooter,arm));
    robotCommands.put("MoveArmSpeaker", new MoveArmSpeaker(arm));
    robotCommands.put("MoveArmSpeaker", new MoveArmIntake(arm));
    robotCommands.put("ShootNoteSpeaker", new ShootNoteSpeaker(shooter, arm));
    robotCommands.put("ScoreAmp", new ScoreAmp(shooter, arm));
    robotCommands.put("ScoreAmp", new AutoAim(drivetrain, aprilCam, arm, shooter, robotToCam));
    NamedCommands.registerCommands(robotCommands);
    configureBindings();
    
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Top Front");
    //return Commands.print("No autonomous command configured");
  }
}
