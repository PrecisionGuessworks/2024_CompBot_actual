// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Paths;
import java.util.HashMap;
import java.util.Map;

import org.photonvision.PhotonCamera;

//import org.photonvision.PhotonCamera;

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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.autoCommands.AutoIntake;
import frc.robot.autoCommands.PathFollowWithEvents;
import frc.robot.commands.AutoAimPose;
import frc.robot.commands.EjectPiece;
import frc.robot.commands.IntakePiece;
import frc.robot.commands.MoveArmAmp;
import frc.robot.commands.MoveArmSpeaker;
import frc.robot.commands.MoveClimber;
import frc.robot.commands.ScoreAmp;
import frc.robot.commands.SetClimberSensorMax;
import frc.robot.commands.SetClimberSensorMin;
import frc.robot.commands.MoveArmIntake;
import frc.robot.commands.MoveArmIntakeAmp;
import frc.robot.commands.ShootNoteSpeaker;
import frc.robot.commands.ShootNoteSpeakerTogether;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PresPoseEstimator;
//import frc.robot.subsystems.PresPoseEstimator;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.Blinkin;
import frc.robot.subsystems.Blinkin.BlinkinSubsystem;
import frc.robot.subsystems.Blinkin.Colors;

public class RobotContainer {
  final double MaxSpeed = 5.0292; // 6 meters per second desired top speed
  final double MaxAngularRate = 2 * Math.PI; // Half a rotation per second max angular velocity

  private final SendableChooser<Command> chooser;

  /* Setting up bindings for necessary control of the swerve drive platform */
  //CommandPS4Controller joystick = new CommandPS4Controller(0);
  public final XboxController joystick = new XboxController(0); // My joystick
  public final XboxController operator = new XboxController(1); //operator

  //private final SendableChooser<Command> autoChooser;

  CommandSwerveDrivetrain drivetrain = Constants.Swerve.TunerConstants.DriveTrain; // My drivetrain
  SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                                                            // driving in open loop
  SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  Telemetry logger = new Telemetry(MaxSpeed);

  //enable for testing once

  PhotonCamera aprilCam = new PhotonCamera("OV2311");



  Transform3d robotToCam = new Transform3d(new Translation3d(0, 0.44, 0.37), new Rotation3d(0,Units.degreesToRadians(15),0));
  Transform3d camToRobot = new Transform3d(new Translation3d(0, -0.44, -0.37), new Rotation3d(0,Units.degreesToRadians(-15),0));

  //Subsystems
  IntakeSubsystem intake = new IntakeSubsystem();
  ShooterSubsystem shooter = new ShooterSubsystem();
  ArmSubsystem arm = new ArmSubsystem();
  ClimberSubsystem climber = new ClimberSubsystem();
  BlinkinSubsystem blinkin = new BlinkinSubsystem();


/*  enable for testing once    */

  PresPoseEstimator poseEstimator = new PresPoseEstimator(aprilCam, drivetrain, robotToCam, camToRobot);

  Map<String, Command> robotCommands  = new HashMap<String, Command>();

  

  //NamedCommands.registerCommand();

  private Command runAuto = drivetrain.getAutoPath("CommandTest");

  public RobotContainer() {
    robotCommands.put("IntakePiece", new IntakePiece(intake, shooter,arm).withTimeout(2.5));
    robotCommands.put("MoveArmSpeaker", new MoveArmSpeaker(arm));
    robotCommands.put("MoveArmIntake", new MoveArmIntake(arm));
    robotCommands.put("ShootNoteSpeaker", new ShootNoteSpeaker(shooter, arm).withTimeout(2.5));
    robotCommands.put("ShootNoteSpeakerTogether", new ShootNoteSpeakerTogether(shooter, arm, intake  ).withTimeout(2.6));
    robotCommands.put("ScoreAmp", new ScoreAmp(shooter, arm, intake));
    
    NamedCommands.registerCommands(robotCommands);

    
    chooser = AutoBuilder.buildAutoChooser("default");
        SmartDashboard.putData("Auto Choices", chooser);
    configureBindings();
    
  }
  
  private final Trigger rightTrigger = new Trigger(() -> joystick.getRightTriggerAxis() > 0.2);
  private final Trigger leftTrigger = new Trigger(() -> joystick.getLeftTriggerAxis() > 0.2);
  private final Trigger operatorLeftTrigger = new Trigger(() -> operator.getLeftTriggerAxis() > 0.2);
  private final Trigger operatorRightTrigger = new Trigger(() -> operator.getRightTriggerAxis() > 0.2);

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

  private final JoystickButton operatorBumperRight =
      new JoystickButton(operator, XboxController.Button.kRightBumper.value);

  private final JoystickButton operatorBumperLeft =
      new JoystickButton(operator, XboxController.Button.kLeftBumper.value);

  private final JoystickButton operatorButtonA =
      new JoystickButton(operator, XboxController.Button.kA.value);
  
      private final JoystickButton operatorButtonY =
      new JoystickButton(operator, XboxController.Button.kY.value);

      private final JoystickButton operatorButtonB =
      new JoystickButton(operator, XboxController.Button.kB.value);

      private final Trigger operatorDPadDown = new POVButton(operator, 180);

      private final Trigger operatorDPadUp = new POVButton(operator, 0);
  

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
    operatorBumperLeft.whileTrue(new EjectPiece(shooter, arm, intake));

    bumperRight.onTrue(new ShootNoteSpeakerTogether(shooter, arm, intake));
    //bumperRight.onFalse(new MoveArmIntake(arm));
    //intake piece
    rightTrigger.whileTrue(new IntakePiece(intake, shooter, arm));

    operatorButtonA.onTrue(new MoveArmIntake(arm));

    //move arm
    operatorBumperRight.onTrue(new ScoreAmp(shooter, arm, intake));
    operatorButtonY.onTrue(new MoveArmAmp(arm));
    operatorDPadDown.whileTrue(new SetClimberSensorMax(climber));
    operatorDPadUp.whileTrue(new SetClimberSensorMin(climber));

    operatorRightTrigger.whileTrue(drivetrain.AutoAim());

    //buttonB.whileTrue(drivetrain.followTrajectoryCommand());

    operatorLeftTrigger.whileTrue(new MoveClimber(climber, operator));

    //climber.setDefaultCommand(new MoveClimber(climber, operator.getRightY(), operator.getLeftY()));
    
    
    
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  

  public Command getAutonomousCommand() {
    
    //return new ShootNoteSpeaker(shooter, arm);
    //return redAuto();
    //return blueAuto();
    //return blueAutoAmp();
    //return redAutoAmp();
    //return new PathPlannerAuto("MidFront");
    return chooser.getSelected();
  }


  
  public Command blueAuto() {
    Pose2d waypoint1 = new Pose2d(1.34, 5.54, new Rotation2d(Units.degreesToRadians(180)));
    Pose2d waypoint2 = new Pose2d(3.0, 7, new Rotation2d(Units.degreesToRadians(0)));

    drivetrain.seedFieldRelative(waypoint1);

    return new SequentialCommandGroup(new ShootNoteSpeaker(shooter, arm).withTimeout(2.5),
     new MoveArmIntake(arm).withTimeout(3.5), 
     new ParallelCommandGroup(new AutoIntake(intake,shooter, arm), drivetrain.followTrajectoryCommand(waypoint2, 0.3)).withTimeout(3.0), 
     drivetrain.followTrajectoryCommand(waypoint1, 0.0), new ShootNoteSpeaker(shooter, arm));

    // new AutoIntake(intake, shooter, arm).withTimeout(3.0);


  }

  public Command redAuto() {
    //Todo change pose points.
    Pose2d waypoint1 = new Pose2d(1.34, 5.54, new Rotation2d(Units.degreesToRadians(0)));
    Pose2d waypoint2 = new Pose2d(3.0, 7, new Rotation2d(Units.degreesToRadians(180)));

    drivetrain.seedFieldRelative(waypoint1);

    return new SequentialCommandGroup(new ShootNoteSpeaker(shooter, arm).withTimeout(2.5),
     new MoveArmIntake(arm).withTimeout(3.5), 
     new ParallelCommandGroup(new AutoIntake(intake,shooter, arm), drivetrain.followTrajectoryCommand(waypoint2, 0.3)).withTimeout(3.0), 
     drivetrain.followTrajectoryCommand(waypoint1, 0.0), new ShootNoteSpeaker(shooter, arm));

    // new AutoIntake(intake, shooter, arm).withTimeout(3.0);


  }
  public Command blueAutoCenter() {
    Pose2d waypoint1 = new Pose2d(1.34, 5.54, new Rotation2d(Units.degreesToRadians(180)));
    Pose2d waypoint2 = new Pose2d(3, 5.54, new Rotation2d(0));
    Pose2d waypoint3 = new Pose2d(3.0, 5.54, new Rotation2d(0));
    Pose2d waypoint4 = new Pose2d(1.6, 5.54, new Rotation2d(0));
    drivetrain.seedFieldRelative(waypoint1);
    
    return new SequentialCommandGroup(new ShootNoteSpeaker(shooter, arm).withTimeout(1.5),
     new MoveArmIntake(arm).withTimeout(3), 
     new ParallelCommandGroup(new AutoIntake(intake,shooter, arm), drivetrain.followTrajectoryCommand(waypoint4, 0.1)).withTimeout(1.5),
     new ParallelCommandGroup(new AutoIntake(intake,shooter, arm), drivetrain.followTrajectoryCommand(waypoint2, 0.1)).withTimeout(1), 
     //old drivetrain.followTrajectoryCommand(waypoint1, 0.0), new ShootNoteSpeaker(shooter, arm));
     new ParallelCommandGroup(drivetrain.followTrajectoryCommand(waypoint1, 0.1)).withTimeout(2.5), 
     new ShootNoteSpeaker(shooter, arm).withTimeout(1.5),
     new MoveArmIntake(arm).withTimeout(3),
     new ParallelCommandGroup(new AutoIntake(intake,shooter, arm), drivetrain.followTrajectoryCommand(waypoint3, 0.1)).withTimeout(2.5)
    );

     
    // new AutoIntake(intake, shooter, arm).withTimeout(3.0);


  }
  public Command blueAutoAmp() {
    Pose2d waypointStart = new Pose2d(1, 2, new Rotation2d(Units.degreesToRadians(-120)));
    Pose2d waypointIntake = new Pose2d(3.88, 2.35, new Rotation2d(0));
    Pose2d waypointRun = new Pose2d(6, 2.35, new Rotation2d(0));
    Pose2d waypoint4 = new Pose2d(2, 2.35, new Rotation2d(0));
    drivetrain.seedFieldRelative(waypointStart);
    
    return new SequentialCommandGroup(new ShootNoteSpeakerTogether(shooter, arm,intake).withTimeout(1.5),
     new MoveArmIntake(arm).withTimeout(2.5), 
     new ParallelCommandGroup(new AutoIntake(intake,shooter, arm), drivetrain.followTrajectoryCommand(waypoint4, 0.5)).withTimeout(1.5),
     new ParallelCommandGroup(new AutoIntake(intake,shooter, arm), drivetrain.followTrajectoryCommand(waypointIntake, 0.1)).withTimeout(1.5),
     drivetrain.followTrajectoryCommand(waypointStart, 0.5).withTimeout(2),
     new ShootNoteSpeakerTogether(shooter, arm,intake).withTimeout(1.5),
     drivetrain.followTrajectoryCommand(waypointRun, 0.5).withTimeout(2)
     
     
     
     
     );
    
     
    // new AutoIntake(intake, shooter, arm).withTimeout(3.0);


  }

  public Command redAutoAmp() {
    Pose2d waypoint1 = new Pose2d(1, 2.35, new Rotation2d(Units.degreesToRadians(120)));
    Pose2d waypoint2 = new Pose2d(3.88, 2, new Rotation2d(0));
    Pose2d waypoint3 = new Pose2d(3.88, 2, new Rotation2d(0));
    Pose2d waypoint4 = new Pose2d(2, 2, new Rotation2d(0));
    drivetrain.seedFieldRelative(waypoint1);
    
    return new SequentialCommandGroup(new ShootNoteSpeaker(shooter, arm).withTimeout(1.5),
     new MoveArmIntake(arm).withTimeout(2.5), 
     new ParallelCommandGroup(new AutoIntake(intake,shooter, arm), drivetrain.followTrajectoryCommand(waypoint4, 0.5)).withTimeout(2),
     new ParallelCommandGroup(new AutoIntake(intake,shooter, arm), drivetrain.followTrajectoryCommand(waypoint2, 0.1)).withTimeout(2), 
     //old drivetrain.followTrajectoryCommand(waypoint1, 0.0), new ShootNoteSpeaker(shooter, arm));
     new ParallelCommandGroup(drivetrain.followTrajectoryCommand(waypoint1, 0.1)).withTimeout(2.5), 
     new ShootNoteSpeaker(shooter, arm).withTimeout(1.5),
     new MoveArmIntake(arm).withTimeout(3),
     new ParallelCommandGroup(new AutoIntake(intake,shooter, arm), drivetrain.followTrajectoryCommand(waypoint3, 0.1)).withTimeout(2.5)
    );
  }
  public Command redAutoCenter() {
    //Todo change pose points.
    //real y is 15.2 but this is just for testing
    Pose2d waypoint1 = new Pose2d(1.34, 5.54, new Rotation2d(Units.degreesToRadians(180)));
    Pose2d waypoint2 = new Pose2d(2.5, 5.54, new Rotation2d(0));
    Pose2d waypoint3 = new Pose2d(3.0, 5.54, new Rotation2d(0));
    Pose2d waypoint4 = new Pose2d(2, 5.54, new Rotation2d(0));
    drivetrain.seedFieldRelative(waypoint1);
    
    return new SequentialCommandGroup(new ShootNoteSpeaker(shooter, arm).withTimeout(1.5),
     new MoveArmIntake(arm).withTimeout(3), 
     new ParallelCommandGroup(new AutoIntake(intake,shooter, arm), drivetrain.followTrajectoryCommand(waypoint4, 0.1)).withTimeout(1.5),
     new ParallelCommandGroup(new AutoIntake(intake,shooter, arm), drivetrain.followTrajectoryCommand(waypoint2, 0.1)).withTimeout(1), 
     //old drivetrain.followTrajectoryCommand(waypoint1, 0.0), new ShootNoteSpeaker(shooter, arm));
     new ParallelCommandGroup(drivetrain.followTrajectoryCommand(waypoint1, 0.1)).withTimeout(2.5), 
     new ShootNoteSpeaker(shooter, arm).withTimeout(1.5),
     new MoveArmIntake(arm).withTimeout(3),
     new ParallelCommandGroup(new AutoIntake(intake,shooter, arm), drivetrain.followTrajectoryCommand(waypoint3, 0.1)).withTimeout(2.5)

    );

    //old Pose2d waypoint1 = new Pose2d(1.34, 5.54, new Rotation2d(0));
    // Pose2d waypoint2 = new Pose2d(3.0, 7, new Rotation2d(Units.degreesToRadians(180)));

    // drivetrain.seedFieldRelative(waypoint1);

    // return new SequentialCommandGroup(new ShootNoteSpeaker(shooter, arm).withTimeout(2.5),
    //  new MoveArmIntake(arm).withTimeout(3.5), 
    //  new ParallelCommandGroup(new AutoIntake(intake,shooter, arm), drivetrain.followTrajectoryCommand(waypoint2, 0.3)).withTimeout(3.0), 
    //  drivetrain.followTrajectoryCommand(waypoint1, 0.0), new ShootNoteSpeaker(shooter, arm));

    // new AutoIntake(intake, shooter, arm).withTimeout(3.0);


  }
}
