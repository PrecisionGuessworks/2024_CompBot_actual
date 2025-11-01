// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.Drive.DRotation;
import static frc.robot.Constants.Drive.DriveDeadband;
import static frc.robot.Constants.Drive.IRotation;
import static frc.robot.Constants.Drive.MaxAngularRatePercentage;
import static frc.robot.Constants.Drive.MaxSpeedPercentage;
import static frc.robot.Constants.Drive.PRotation;
import static frc.robot.Constants.Drive.RotationDeadband;
import static frc.robot.Constants.Drive.SnapDriveDeadband;
import static frc.robot.Constants.Drive.SnapRotationDeadband;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import org.json.simple.parser.ParseException;
import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.quixlib.viz.Link2d;
import frc.quixlib.viz.Viz2d;
import frc.robot.commands.Intake;
import frc.robot.commands.QuickAmpStow;
import frc.robot.commands.QuickScore;
import frc.robot.commands.StowArm;
import frc.robot.generated.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

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

// Climber viz
private static final Link2d climberFrameViz =
robotViz.addLink(
    new Link2d(
        robotViz,
        "Climber Base",
        Constants.Viz.climberBaseLength,
        4.0,
        Color.kGreen,
        new Transform2d(
            Constants.Viz.climberBaseX,
            Constants.Viz.climberBaseY,
            Constants.Viz.climberAngle)));
private static final Link2d climberCarriageViz =
climberFrameViz.addLink(
    new Link2d(
        robotViz,
        "Climber Carriage",
        Constants.Viz.climberCarriageLength,
        6.0,
        Color.kLightGreen));


// Intake viz
private static final Link2d intakeRollerViz =
robotViz.addLink(
    new Link2d(robotViz, "Intake Roller", Units.inchesToMeters(1.0), 10.0, Color.kLightBlue));


private static final Link2d ArmArmViz =
robotViz.addLink(
        new Link2d(robotViz, "Arm Arm", Constants.Viz.ArmArmLength, 10, Color.kRed));

private static final Link2d ArmWheelViz =
ArmArmViz.addLink(
        new Link2d(robotViz, "Arm Wheel", Units.inchesToMeters(2.0), 10, Color.kGreen));
private static final Link2d ArmFeederVoz =
ArmArmViz.addLink(
        new Link2d(robotViz, "Arm Feeder", Units.inchesToMeters(2.0), 10, Color.kGreen));
private static final Link2d ArmShotUp =
ArmArmViz.addLink(
        new Link2d(robotViz, "Arm Shooter Up", Units.inchesToMeters(3.0), 10, Color.kOrange));
private static final Link2d ArmShotLow =
ArmArmViz.addLink(
        new Link2d(robotViz, "Arm Shooter Low", Units.inchesToMeters(3.0), 10, Color.kOrange));


    



    public static final IntakeSubsystem intake = new IntakeSubsystem(intakeRollerViz);
    public static final ArmSubsystem arm = new ArmSubsystem(ArmArmViz,ArmWheelViz, ArmFeederVoz, ArmShotUp, ArmShotLow);
    public static final ClimberSubsystem climber = new ClimberSubsystem(climberCarriageViz);



    public RobotContainer() {
        if (Constants.Logging){
        // Starts recording to data log
        DataLogManager.start();
        // Record both DS control and joystick data
        DriverStation.startDataLog(DataLogManager.getLog());
        }
        // drivetrain.setStateStdDevs(Constants.Vision.ODOM_STD_DEV);
        


        //robotCommands.put("IntakePiece", new IntakeAlgae(intake,1).withTimeout(2.5));
        robotCommands.put("QuickScore", new QuickScore(arm));
        //robotCommands.put("IntakeCoralRollerFast",Commands.runOnce(() -> RobotContainer.arm.setRollerVelocityandCurrent(Constants.Arm.intakeVelocity,55,90)));
        robotCommands.put("StowArm", new StowArm(arm,intake));

    
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

        driver.rightTrigger().whileTrue(new Intake(intake, arm));
        driver.rightBumper().whileTrue(new QuickScore(arm));
        driver.rightTrigger().onFalse(new QuickAmpStow(arm));
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
      //driver.x().whileTrue(new StowArm(elevator, arm));

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
