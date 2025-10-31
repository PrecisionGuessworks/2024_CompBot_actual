// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import org.json.simple.parser.ParseException;
import org.photonvision.PhotonUtils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.quixlib.motorcontrol.PIDConfig;
import frc.quixlib.viz.Link2d;
import frc.quixlib.viz.Viz2d;
import frc.robot.commands.MoveupArm;
import frc.robot.commands.StowArm;
import frc.robot.Constants.Climber;
import frc.robot.commands.ClimbSet;
import frc.robot.commands.ClimbZero;
import frc.robot.commands.QuickScore;
import frc.robot.commands.MoveStow;
import frc.robot.commands.Intake;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

import static frc.robot.Constants.Drive.*;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

public class RobotContainer {
    private double MaxSpeed = MaxSpeedPercentage*(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(MaxAngularRatePercentage).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public final CommandXboxController driver = new CommandXboxController(0);
    public final CommandXboxController operator = new CommandXboxController(1);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public static final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain(TunerConstants.DrivetrainConstants,250, Constants.Vision.ODOM_STD_DEV, Constants.Vision.kSingleTagStdDevs, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);

    private Optional<Alliance> m_ally = DriverStation.getAlliance();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * DriveDeadband).withRotationalDeadband(MaxAngularRate * RotationDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.FieldCentric drivelineup = new SwerveRequest.FieldCentric()
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.FieldCentric driveAuto = new SwerveRequest.FieldCentric()
            .withDeadband(SnapDriveDeadband).withRotationalDeadband(SnapRotationDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

            private final SwerveRequest.FieldCentricFacingAngle angle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * SnapRotationDeadband).withRotationalDeadband(MaxAngularRate * SnapRotationDeadband) // Add a deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors 
          //  .withSteerRequestType(SteerRequestType.MotionMagicExpo); // Use motion magic control for steer motors

    private PowerDistribution powerDistribution = new PowerDistribution();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;


Map<String, Command> robotCommands  = new HashMap<String, Command>();



private static  final Viz2d robotViz =
      new Viz2d("Robot Viz", Units.inchesToMeters(80.0), Units.inchesToMeters(120.0), 1.0);

private static  final Link2d chassisViz =
      robotViz.addLink(
          new Link2d(
              robotViz,
              "Chassis",
              Units.inchesToMeters(29.0),
              30.0,
              new Color("#FAB604"),
              new Transform2d(Constants.Viz.xOffset, Units.inchesToMeters(3.0), new Rotation2d())));

  // Elevator viz
  private  static final Link2d elevatorFrameViz =
      robotViz.addLink(
          new Link2d(
              robotViz,
              "Elevator Base",
              Constants.Viz.elevatorBaseLength,
              4.0,
              Color.kGreen,
              new Transform2d(
                  Constants.Viz.elevatorBaseX,
                  Constants.Viz.elevatorBaseY,
                  Constants.Viz.elevatorAngle)));
  private static  final Link2d elevatorCarriageViz =
      elevatorFrameViz.addLink(
          new Link2d(
              robotViz,
              "Elevator Carriage",
              Constants.Viz.elevatorCarriageLength,
              6.0,
              Color.kLightGreen));
// Intake viz
// private static final Link2d intakeArmViz =
// robotViz.addLink(
//     new Link2d(robotViz, "Intake Arm", Constants.Viz.intakeArmLength, 10.0, Color.kBlue));
// private static final Link2d intakeRollerViz =
// intakeArmViz.addLink(
//     new Link2d(robotViz, "Intake Roller", Units.inchesToMeters(1.0), 10.0, Color.kLightBlue));


private static final Link2d ArmArmViz =
elevatorCarriageViz.addLink(
        new Link2d(robotViz, "Arm Arm", Constants.Viz.ArmArmLength, 10, Color.kRed));
private static final Link2d ArmWristViz =
ArmArmViz.addLink(
        new Link2d(robotViz, "Arm Wrist", Constants.Viz.ArmWristLength, 10, Color.kOrange));
private static final Link2d ArmWheelViz =
ArmWristViz.addLink(
        new Link2d(robotViz, "Arm Wheel", Units.inchesToMeters(2.0), 10, Color.kCoral));

    
// Climber viz
// private static final Link2d climberFrameViz =
// robotViz.addLink(
//     new Link2d(
//         robotViz,
//         "Climber Base",
//         Constants.Viz.climberBaseLength,
//         4.0,
//         Color.kGreen,
//         new Transform2d(
//             Constants.Viz.climberBaseX,
//             Constants.Viz.climberBaseY,
//             Constants.Viz.climberAngle)));
// private static final Link2d climberCarriageViz =
// climberFrameViz.addLink(
//     new Link2d(
//         robotViz,
//         "Climber Carriage",
//         Constants.Viz.climberCarriageLength,
//         6.0,
//         Color.kLightGreen));


        //public static final IntakeSubsystem intake = new IntakeSubsystem(intakeArmViz, intakeRollerViz);
        public static final ArmSubsystem arm = new ArmSubsystem(ArmArmViz,ArmWheelViz);
//        public static final ClimberSubsystem climber = new ClimberSubsystem(climberCarriageViz);








    public RobotContainer() {
        if (Constants.Logging){
        // Starts recording to data log
        DataLogManager.start();
        // Record both DS control and joystick data
        DriverStation.startDataLog(DataLogManager.getLog());
        }
        // drivetrain.setStateStdDevs(Constants.Vision.ODOM_STD_DEV);
        


        //robotCommands.put("IntakePiece", new IntakeAlgae(intake,1).withTimeout(2.5));
        robotCommands.put("CoralMoveScore", new QuickScore(elevator, arm));
        robotCommands.put("IntakeCoralRollerFast",Commands.runOnce(() -> RobotContainer.arm.setRollerVelocityandCurrent(Constants.Arm.intakeVelocity,55,90)));
        robotCommands.put("StowArm", new StowArm(arm));

    
        NamedCommands.registerCommands(robotCommands);


        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto", autoChooser);
        angle.HeadingController.setPID( PRotation,  IRotation , DRotation);
        configureBindings();

    


        SmartDashboard.putData(
        "Gyro",
        builder -> {
          builder.setSmartDashboardType("Gyro");
          builder.addDoubleProperty("Value", () -> drivetrain.getPigeon2().getYaw().getValueAsDouble(), null);
        });
         // SmartDashboard.putNumber("Time",Timer.getMatchTime());
          SmartDashboard.putNumber("Time",DriverStation.getMatchTime());
          if(Constants.ExtraInfo){
          SmartDashboard.putNumber("Voltage",RobotController.getBatteryVoltage());
          SmartDashboard.putNumber("CAN",RobotController.getCANStatus().percentBusUtilization * 100.0);
          SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
        SmartDashboard.putData("Power Distribution Panel", powerDistribution);
          }

        //PathfindingCommand.warmupCommand().ignoringDisable(true).schedule();;
        
    }

    

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        //driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // driver.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        // ));

        driver.leftBumper().whileTrue(new ParallelCommandGroup(new QuickScore(elevator, arm), pathfindingCommand(true)));
        driver.rightBumper().whileTrue(new ParallelCommandGroup(new QuickScore(elevator, arm), pathfindingCommand(false)));
        driver.leftBumper().onFalse(new MoveStow(elevator, arm));
        driver.rightBumper().onFalse(new MoveStow(elevator, arm));

        //driver.y().whileTrue(new ClimbSet(climber));
        //driver.x().whileTrue(pathfindingtofollowCommand());

        // Old
        driver.leftTrigger().or(() -> (RobotContainer.elevator.getHeight() >= Constants.Elevator.SlowmodeHeight) && !DriverStation.isAutonomous() && !LineupCommand).whileTrue(drivetrain.applyRequest(() ->
        drive.withVelocityX(-driver.getLeftY() * MaxSpeed * Constants.Drive.SlowSpeedPercentage) // Drive forward with negative Y (forward)
            .withVelocityY(-driver.getLeftX() * MaxSpeed * Constants.Drive.SlowSpeedPercentage) // Drive left with negative X (left)
            .withRotationalRate(-driver.getRightX() * MaxAngularRate * Constants.Drive.SlowRotPercentage) // Drive counterclockwise with negative X (left)
        ));

        // Test not work
        // driver.leftBumper().or(driver.rightBumper().or(driver.leftTrigger().or(() -> (RobotContainer.elevator.getHeight() >= Constants.Elevator.SlowmodeHeight) && !DriverStation.isAutonomous()).whileTrue(drivetrain.applyRequest(() ->
        // drive.withVelocityX(-driver.getLeftY() * MaxSpeed * Constants.Drive.SlowSpeedPercentage) // Drive forward with negative Y (forward)
        //     .withVelocityY(-driver.getLeftX() * MaxSpeed * Constants.Drive.SlowSpeedPercentage) // Drive left with negative X (left)
        //     .withRotationalRate(-driver.getRightX() * MaxAngularRate * Constants.Drive.SlowRotPercentage) // Drive counterclockwise with negative X (left)
        // ))));

        driver.rightTrigger().whileTrue(new Intake(elevator, arm));
       // driver.leftTrigger().whileTrue(new IntakeAlgae(intake, 0));
        driver.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        

        driver.pov(90).whileTrue(drivetrain.applyRequest(() ->
            angle.withVelocityX(-driver.getLeftY() * MaxSpeed)
            .withVelocityY(-driver.getLeftX() * MaxSpeed)
            .withTargetDirection(new Rotation2d(Math.toRadians(0))))
        );
        driver.pov(180).whileTrue(drivetrain.applyRequest(() ->
            angle.withVelocityX(-driver.getLeftY() * MaxSpeed)
            .withVelocityY(-driver.getLeftX() * MaxSpeed)
            .withTargetDirection(new Rotation2d(Math.toRadians(-90))))
        );
        driver.pov(270).whileTrue(drivetrain.applyRequest(() ->
            angle.withVelocityX(-driver.getLeftY() * MaxSpeed)
            .withVelocityY(-driver.getLeftX() * MaxSpeed)
            .withTargetDirection(new Rotation2d(Math.toRadians(-180))))
        );
        driver.pov(0).whileTrue(drivetrain.applyRequest(() ->
            angle.withVelocityX(-driver.getLeftY() * MaxSpeed)
            .withVelocityY(-driver.getLeftX() * MaxSpeed)
            .withTargetDirection(new Rotation2d(Math.toRadians(-270))))
        );


       // driver.a().whileTrue(new AlgeaWack(elevator, arm));
       
       driver.a().whileTrue(new MoveupArm(1,elevator,arm)); 
       driver.b().whileTrue(new MoveupArm(2,elevator,arm)); 
       driver.y().whileTrue(new Moveup(elevator));
      driver.x().whileTrue(new AlgeaWack(elevator, arm));
      //driver.x().whileTrue(new StowArm(elevator, arm));

       operator.y().whileTrue(new CoralEleUp(elevator));
       operator.a().whileTrue(new AlgeaWack(elevator, arm));
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a - single log.
        // driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        
     //   operator.leftTrigger().whileTrue(new IntakeAlgae(intake, 2));
     //   operator.rightTrigger().whileTrue(new IntakeAlgae(intake, 1));
        operator.rightBumper().whileTrue(new StowArm(elevator, arm));
        operator.start().whileTrue(drivetrain.applyRequest(() ->
        drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
    ));

        
        



        //drivetrain.registerTelemetry(logger::telemeterize);
    }










// --------------------------------------------------------- Commands --------------------------------------------------------- 




    public Command getAutonomousCommand() {
        /* First put the drivetrain into auto run mode, then run the auto */
        return autoChooser.getSelected();
    }

    public Rotation2d targetangle() {
        /* First put the drivetrain into auto run mode, then run the auto */
        SwerveDriveState state = drivetrain.getState();
        Pose2d pose = state.Pose;
        pose = new Pose2d(pose.getTranslation(), new Rotation2d(0));
        Pose2d targetpose = new Pose2d(16.7,5.5,new Rotation2d(0));
        System.out.println(PhotonUtils.getYawToPose(pose,targetpose));
        return PhotonUtils.getYawToPose(pose,targetpose);
        
    }


        
     private Command pathfindingtofollowCommand() {
        // Since we are using a holonomic drivetrain, the rotation component of this pose
        // represents the goal holonomic rotation
        PathPlannerPath path = null;
        try {
            path = PathPlannerPath.fromPathFile("Testpath");
        } catch (FileVersionException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                4.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindThenFollowPath(
                path,
                constraints
                 
        );
    }
}
//Abhi was here
