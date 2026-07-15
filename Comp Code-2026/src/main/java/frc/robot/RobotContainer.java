// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.IntakePiece;
import frc.robot.commands.MoveArmIntake;
import frc.robot.commands.MoveArmSpeaker;
import frc.robot.commands.ScoreAmp;
import frc.robot.commands.ShootNoteSpeaker;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.PresPoseEstimator;
//import frc.robot.subsystems.PresPoseEstimator;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  final static double MaxSpeed = 5.0292; // 6 meters per second desired top speed
  final static double MaxAngularRate = 2 * Math.PI; // Half a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  //CommandPS4Controller joystick = new CommandPS4Controller(0);
  public final XboxController joystick = new XboxController(0); // My joystick
  // public final XboxController operator = new XboxController(1); //operator

  //private final SendableChooser<Command> autoChooser;

  public static final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain(TunerConstants.DrivetrainConstants,250, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);
  // SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * Constants.Drive.DriveDeadband).withRotationalDeadband(MaxAngularRate * Constants.Drive.RotationDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);                                                                                          // driving in open loop
  // SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  // SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  // Telemetry logger = new Telemetry(MaxSpeed);

  //enable for testing once

  //PhotonCamera aprilCam = new PhotonCamera("OV2311");



  Transform3d robotToCam = new Transform3d(new Translation3d(0.0, 0.44, 0.37), new Rotation3d(0,Units.degreesToRadians(15),0));
  Transform3d camToRobot = new Transform3d(new Translation3d(0.0, -0.44, -0.37), new Rotation3d(0,Units.degreesToRadians(-15),0));

  //Subsystems
  IntakeSubsystem intake = new IntakeSubsystem();
  ShooterSubsystem shooter = new ShooterSubsystem();
  ArmSubsystem arm = new ArmSubsystem();
  ClimberSubsystem climber = new ClimberSubsystem();


/*  enable for testing once    */

  //PresPoseEstimator poseEstimator = new PresPoseEstimator(aprilCam, drivetrain, robotToCam, camToRobot);

  Map<String, Command> robotCommands  = new HashMap<String, Command>();

  

  //NamedCommands.registerCommand();

  // private Command runAuto = drivetrain.getAutoPath("CommandTest");

  
  
  public static final CommandXboxController driver = new CommandXboxController(0);
  

  private void configureBindings() {
    drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

    // buttonY.whileTrue(drivetrain.applyRequest(() -> brake));
    // buttonA.whileTrue(drivetrain
    //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    

    // reset the field-centric heading on left bumper press
    // bumperLeft.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    driver.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    //shoot da note
    // operatorBumperLeft.whileTrue(new EjectPiece(shooter, arm, intake));

    driver.rightBumper().whileTrue(new ShootNoteSpeaker(shooter, arm));
    //bumperRight.onFalse(new MoveArmIntake(arm));
    //intake piece
    driver.rightTrigger().whileTrue(new IntakePiece(intake, shooter, arm));

    // operatorButtonA.whileTrue(new MoveArmIntake(arm));

    //move arm
    // operatorBumperRight.whileTrue(new ScoreAmp(shooter, arm, intake));
    // operatorButtonY.whileTrue(new MoveArmAmp(arm));
    // operatorDPadDown.whileTrue(new SetClimberSensorMax(climber));
    // operatorDPadUp.whileTrue(new SetClimberSensorMin(climber));

    // //buttonB.whileTrue(drivetrain.followTrajectoryCommand());

    // operatorLeftTrigger.whileTrue(new MoveClimber(climber, operator));

    //climber.setDefaultCommand(new MoveClimber(climber, operator.getRightY(), operator.getLeftY()));
    

    // if (Utils.isSimulation()) {
    //   drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    // }
    // drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    robotCommands.put("IntakePiece", new IntakePiece(intake, shooter,arm));
    robotCommands.put("MoveArmSpeaker", new MoveArmSpeaker(arm));
    robotCommands.put("MoveArmSpeaker", new MoveArmIntake(arm));
    robotCommands.put("ShootNoteSpeaker", new ShootNoteSpeaker(shooter, arm));
    robotCommands.put("ScoreAmp", new ScoreAmp(shooter, arm, intake));
    
    NamedCommands.registerCommands(robotCommands);

    //autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    //SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
    
  }

  // public Command getAutonomousCommand() {
    
  //   //return new ShootNoteSpeaker(shooter, arm);
  //   // return blueAutoAmp();
  //   //return redAutoAmp();
  // }
  // public Command blueAutoCenter() {
  //   Pose2d waypoint1 = new Pose2d(1.34, 5.54, new Rotation2d(Units.degreesToRadians(180)));
  //   Pose2d waypoint2 = new Pose2d(3, 5.54, new Rotation2d(0));
  //   Pose2d waypoint3 = new Pose2d(3.0, 5.54, new Rotation2d(0));
  //   Pose2d waypoint4 = new Pose2d(1.6, 5.54, new Rotation2d(0));
  //   drivetrain.seedFieldRelative(waypoint1);
    
  //   return new SequentialCommandGroup(new ShootNoteSpeaker(shooter, arm).withTimeout(1.5),
  //    new MoveArmIntake(arm).withTimeout(3), 
  //    new ParallelCommandGroup(new AutoIntake(intake,shooter, arm), drivetrain.followTrajectoryCommand(waypoint4, 0.1)).withTimeout(1.5),
  //    new ParallelCommandGroup(new AutoIntake(intake,shooter, arm), drivetrain.followTrajectoryCommand(waypoint2, 0.1)).withTimeout(1), 
  //    //old drivetrain.followTrajectoryCommand(waypoint1, 0.0), new ShootNoteSpeaker(shooter, arm));
  //    new ParallelCommandGroup(drivetrain.followTrajectoryCommand(waypoint1, 0.1)).withTimeout(2.5), 
  //    new ShootNoteSpeaker(shooter, arm).withTimeout(1.5),
  //    new MoveArmIntake(arm).withTimeout(3),
  //    new ParallelCommandGroup(new AutoIntake(intake,shooter, arm), drivetrain.followTrajectoryCommand(waypoint3, 0.1)).withTimeout(2.5)
  //   );

     
  //   // new AutoIntake(intake, shooter, arm).withTimeout(3.0);


  // }
  // public Command blueAutoAmp() {
  //   Pose2d waypoint1 = new Pose2d(1, 2, new Rotation2d(Units.degreesToRadians(-120)));
  //   Pose2d waypoint2 = new Pose2d(3.88, 2.35, new Rotation2d(0));
  //   Pose2d waypoint3 = new Pose2d(3.88, 2.35, new Rotation2d(0));
  //   Pose2d waypoint4 = new Pose2d(2, 2.35, new Rotation2d(0));
  //   drivetrain.seedFieldRelative(waypoint1);
    
  //   return new SequentialCommandGroup(new ShootNoteSpeaker(shooter, arm).withTimeout(1.5),
  //    new MoveArmIntake(arm).withTimeout(2.5), 
  //    new ParallelCommandGroup(new AutoIntake(intake,shooter, arm), drivetrain.followTrajectoryCommand(waypoint4, 0.5)).withTimeout(2),
  //    new ParallelCommandGroup(new AutoIntake(intake,shooter, arm), drivetrain.followTrajectoryCommand(waypoint2, 0.1)).withTimeout(2), 
  //    //old drivetrain.followTrajectoryCommand(waypoint1, 0.0), new ShootNoteSpeaker(shooter, arm));
  //    new ParallelCommandGroup(drivetrain.followTrajectoryCommand(waypoint1, 0.1)).withTimeout(2.5), 
  //    new ShootNoteSpeaker(shooter, arm).withTimeout(1.5),
  //    new MoveArmIntake(arm).withTimeout(3),
  //    new ParallelCommandGroup(new AutoIntake(intake,shooter, arm), drivetrain.followTrajectoryCommand(waypoint3, 0.1)).withTimeout(2.5)
  //   );

     
  //   // new AutoIntake(intake, shooter, arm).withTimeout(3.0);


  // }

  // public Command redAutoAmp() {
  //   Pose2d waypoint1 = new Pose2d(1, 2.35, new Rotation2d(Units.degreesToRadians(120)));
  //   Pose2d waypoint2 = new Pose2d(3.88, 2, new Rotation2d(0));
  //   Pose2d waypoint3 = new Pose2d(3.88, 2, new Rotation2d(0));
  //   Pose2d waypoint4 = new Pose2d(2, 2, new Rotation2d(0));
  //   drivetrain.seedFieldRelative(waypoint1);
    
  //   return new SequentialCommandGroup(new ShootNoteSpeaker(shooter, arm).withTimeout(1.5),
  //    new MoveArmIntake(arm).withTimeout(2.5), 
  //    new ParallelCommandGroup(new AutoIntake(intake,shooter, arm), drivetrain.followTrajectoryCommand(waypoint4, 0.5)).withTimeout(2),
  //    new ParallelCommandGroup(new AutoIntake(intake,shooter, arm), drivetrain.followTrajectoryCommand(waypoint2, 0.1)).withTimeout(2), 
  //    //old drivetrain.followTrajectoryCommand(waypoint1, 0.0), new ShootNoteSpeaker(shooter, arm));
  //    new ParallelCommandGroup(drivetrain.followTrajectoryCommand(waypoint1, 0.1)).withTimeout(2.5), 
  //    new ShootNoteSpeaker(shooter, arm).withTimeout(1.5),
  //    new MoveArmIntake(arm).withTimeout(3),
  //    new ParallelCommandGroup(new AutoIntake(intake,shooter, arm), drivetrain.followTrajectoryCommand(waypoint3, 0.1)).withTimeout(2.5)
  //   );
  // }
  // public Command redAutoCenter() {
  //   //Todo change pose points.
  //   //real y is 15.2 but this is just for testing
  //   Pose2d waypoint1 = new Pose2d(1.34, 5.54, new Rotation2d(Units.degreesToRadians(180)));
  //   Pose2d waypoint2 = new Pose2d(2.5, 5.54, new Rotation2d(0));
  //   Pose2d waypoint3 = new Pose2d(3.0, 5.54, new Rotation2d(0));
  //   Pose2d waypoint4 = new Pose2d(2, 5.54, new Rotation2d(0));
  //   drivetrain.seedFieldRelative(waypoint1);
    
  //   return new SequentialCommandGroup(new ShootNoteSpeaker(shooter, arm).withTimeout(1.5),
  //    new MoveArmIntake(arm).withTimeout(3), 
  //    new ParallelCommandGroup(new AutoIntake(intake,shooter, arm), drivetrain.followTrajectoryCommand(waypoint4, 0.1)).withTimeout(1.5),
  //    new ParallelCommandGroup(new AutoIntake(intake,shooter, arm), drivetrain.followTrajectoryCommand(waypoint2, 0.1)).withTimeout(1), 
  //    //old drivetrain.followTrajectoryCommand(waypoint1, 0.0), new ShootNoteSpeaker(shooter, arm));
  //    new ParallelCommandGroup(drivetrain.followTrajectoryCommand(waypoint1, 0.1)).withTimeout(2.5), 
  //    new ShootNoteSpeaker(shooter, arm).withTimeout(1.5),
  //    new MoveArmIntake(arm).withTimeout(3),
  //    new ParallelCommandGroup(new AutoIntake(intake,shooter, arm), drivetrain.followTrajectoryCommand(waypoint3, 0.1)).withTimeout(2.5)

  // );

    //old Pose2d waypoint1 = new Pose2d(1.34, 5.54, new Rotation2d(0));
    // Pose2d waypoint2 = new Pose2d(3.0, 7, new Rotation2d(Units.degreesToRadians(180)));

    // drivetrain.seedFieldRelative(waypoint1);

    // return new SequentialCommandGroup(new ShootNoteSpeaker(shooter, arm).withTimeout(2.5),
    //  new MoveArmIntake(arm).withTimeout(3.5), 
    //  new ParallelCommandGroup(new AutoIntake(intake,shooter, arm), drivetrain.followTrajectoryCommand(waypoint2, 0.3)).withTimeout(3.0), 
    //  drivetrain.followTrajectoryCommand(waypoint1, 0.0), new ShootNoteSpeaker(shooter, arm));

    // new AutoIntake(intake, shooter, arm).withTimeout(3.0);


  // }
}
