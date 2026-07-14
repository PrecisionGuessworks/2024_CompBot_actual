package frc.robot;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.quixlib.devices.CANDeviceID;
import frc.quixlib.motorcontrol.MechanismRatio;
import frc.quixlib.motorcontrol.PIDConfig;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class Constants {
    // CANID's:
    //
    // Drivetrain 1-19
    // Elevator / Arm 20-29
    // Intake 30-39
    // Climber 40-49

    public static final String kCanivoreName = "canivore";
    public static final String kDriveTrainCanivoreName = kCanivoreName;
    public static final String kSuperStructureCanivoreName = kCanivoreName;
    public static final String kRioName = "rio";

    public static final double g = 9.80148; // m/s/s
    public static final double defaultPeriodSecs = 0.02; // s
    public static final double Drag = 0.07; // Drag Coefficient for Simulations
    public static final double Friction = 4; // Friction Coefficient for Simulations
    public static final double Shotefficiency = 0.85;
    public static final boolean isSim = edu.wpi.first.wpilibj.RobotBase.isSimulation(); // Uses diffrent constants if
                                                                                        // sim or real
    public static final boolean SimFuel = isSim; // Set to true to enable fuel simulation
    public static final boolean DogLogEnabled = true; // Set to true to enable DogLog telemetry
    public static final boolean DogLogNetworkTables = true; // Set to true to enable DogLog over NetworkTables
    public static final boolean LogHardware = true; // Set to true to enable hardware logging in DogLog (Should be on
                                                    // unless low on ram/cpu)
    public static final boolean UseRewind = DriverStation.isFMSAttached(); // Records video for robot from the
                                                                           // limelights
    public static boolean Lineup = false; // Auto Lineup to Reef to Scrore.
    public static boolean ExtraInfo = true; // Turn on Extra network info
    public static boolean Logging = false; // Turn on Logging
    public static double feildFlip = 16.5;
    public static double feildFlipy = 5.0;

    public static class Vision {
        public static final String kCameraName = "FrontCamera"; // Front

        // Cam mounted facing forward, half a meter forward of center, half a meter up
        // from center, up 15 degs.
        public static final Transform3d kRobotToCam = new Transform3d(
                new Translation3d(Units.inchesToMeters(13.311564), 0.0, Units.inchesToMeters(7.332072)),
                new Rotation3d(0, Math.toRadians(-25), 0));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
        // public static final AprilTagFieldLayout kTagLayout =
        // AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8); // m, m, rad
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
        public static final Matrix<N3, N1> ODOM_STD_DEV = VecBuilder.fill(0.03, 0.03, Units.degreesToRadians(0.01));

    }

    public static class Drive { // Drive Constants that are not in TunerConstants / Gnenerated

        // PID for Rotation and Translation for Auto and Teleop Snap
        public static final double PTranslation = 5;
        public static final double ITranslation = 0.001;
        public static final double DTranslation = 0.1;

        public static final double PRotation = 5;
        public static final double IRotation = 0.001;
        public static final double DRotation = 0.03;

        // 0.0-1.0 of the max speed
        public static final double MaxSpeedPercentage = 0.95; // Default 1.0
        public static final double SlowSpeedPercentage = 0.10; // Default 0.15

        // Rotation per second max angular velocity
        public static final double MaxAngularRatePercentage = 0.82; // Default 0.75
        public static final double SlowRotPercentage = 0.15; // Default 0.15

        // Deadbands for the drive and rotation
        public static final double DriveDeadband = isSim ? 0.15 : 0.02; // Drive Deadband
        public static final double RotationDeadband = isSim ? 0.15 : 0.02; // Rotation Deadband
        public static final double SnapDriveDeadband = 0.001; // Snap Rotation Deadband
        public static final double SnapRotationDeadband = 0.001; // Snap Rotation Deadband

    }

    public static final class Arm {
        public static final int beamBreakPort = 0;

        public static final CANDeviceID armMotorID = new CANDeviceID(16, kRioName);
        public static final CANDeviceID armCoderID = new CANDeviceID(24, kRioName);
        public static final MechanismRatio armMotorRatio = new MechanismRatio(1, 75);
        public static final MechanismRatio armSensorRatio = new MechanismRatio(1, 1);
        public static final boolean armMotorInvert = false;

        // public static final ArmFeedforward armFeedForward = new ArmFeedforward(3.0,
        // 0.3, 0.6);
        public static final Constraints ArmConstraints = new Constraints(1.5, 0.5); // rad/s and rad/s^2
        public static final double ArmMaxJerk = 1.0; // rad/s^3
        public static final int armPositionPIDSlot = 0;
        public static final PIDConfig armPositionPIDConfig = new PIDConfig(12, 0.00, 0.00);
        public static final double armExpo_kV = 0.0;
        public static final double armExpo_kA = 0.0;

        public static final CANDeviceID armfollowerID = new CANDeviceID(15, kRioName);
        public static final MechanismRatio armfollowerRatio = new MechanismRatio(1, 75);
        public static final MotorAlignmentValue armfollowerInvert = MotorAlignmentValue.Opposed;

        public static final CANDeviceID ampMotorID = new CANDeviceID(18, kRioName);
        public static final MechanismRatio ampMotorRatio = new MechanismRatio(1, 4);
        public static final boolean ampMotorInvert = false;

        public static final SimpleMotorFeedforward ampFeedforward = new SimpleMotorFeedforward(0, 0.019);
        public static final int ampVelocityPIDSlot = 0;
        public static final PIDConfig ampVelocityPIDConfig = new PIDConfig(2.0, 0.0, 0.0);
        public static final int ampPositionPIDSlot = 0;
        public static final PIDConfig ampPositionPIDConfig = new PIDConfig(2.0, 0.0, 0.0);

        public static final CANDeviceID feederMotorID = new CANDeviceID(17, kRioName);
        public static final MechanismRatio feederMotorRatio = new MechanismRatio(1, 4);
        public static final boolean feederMotorInvert = true;

        public static final SimpleMotorFeedforward feederFeedforward = new SimpleMotorFeedforward(0, 0.019);
        public static final int feederVelocityPIDSlot = 0;
        public static final PIDConfig feederVelocityPIDConfig = new PIDConfig(2.0, 0.0, 0.0);
        public static final int feederPositionPIDSlot = 0;
        public static final PIDConfig feederPositionPIDConfig = new PIDConfig(2.0, 0.0, 0.0);

        public static final CANDeviceID shooterUpperMotorID = new CANDeviceID(19, kRioName);
        public static final MechanismRatio shooterUpperMotorRatio = new MechanismRatio(1, 1);
        public static final boolean shooterUpperMotorInvert = true;

        public static final SimpleMotorFeedforward shooterUpperFeedforward = new SimpleMotorFeedforward(0, 0.019);
        public static final int shooterUpperVelocityPIDSlot = 0;
        public static final PIDConfig shooterUpperVelocityPIDConfig = new PIDConfig(2.0, 0.0, 0.0);
        public static final int shooterUpperPositionPIDSlot = 0;
        public static final PIDConfig shooterUpperPositionPIDConfig = new PIDConfig(2.0, 0.0, 0.0);

        public static final CANDeviceID shooterLowerMotorID = new CANDeviceID(20, kRioName);
        public static final MechanismRatio shooterLowerMotorRatio = new MechanismRatio(1, 1);
        public static final boolean shooterLowerMotorInvert = true;

        public static final SimpleMotorFeedforward shooterLowerFeedforward = new SimpleMotorFeedforward(0, 0.019);
        public static final int shooterLowerVelocityPIDSlot = 0;
        public static final PIDConfig shooterLowerVelocityPIDConfig = new PIDConfig(2.0, 0.0, 0.0);
        public static final int shooterLowerPositionPIDSlot = 0;
        public static final PIDConfig shooterLowerPositionPIDConfig = new PIDConfig(2.0, 0.0, 0.0);

        // TODO: Use real values
        public static final double armBootAbsPositionOffset = Units.degreesToRadians(0);
        public static final double armMinAngle = Units.degreesToRadians(-25.0); // rads
        public static final double armMaxAngle = Units.degreesToRadians(110.0); // rads
        public static final double armStartingAngle = armMinAngle;
        public static final double armCgOffset = Units.degreesToRadians(2);

        public static final double AngleTolerance = Units.degreesToRadians(3);
        public static final double ShootTolerance = 20;

        public static final double intakeVelocity = 80.0; // rads/s
        public static final double outtakeVelocity = -40.0; // rads/s
        public static final double quickShootVelocity = 400.0; // rads/s
        public static final double quickAmpVelocity = 80.0; // rads/s

        public static final double feederIntakeVelocity = 80.0; // rads/s
        public static final double ampIntakeVelocity = -80.0; // rads/s
        public static final double feederShootVelocity = 160.0; // rads/s
        public static final double ampShootVelocity = 80.0; // rads/s

        public static final double rollerStallVelocity = Math.PI * Math.PI * (1.0 / 32.0); // rads/s
        public static final double rollerStallCurrent = 30; // Amps

        public static final double armIntakeAngle = Units.degreesToRadians(-20);
        public static final double armShootAngle = Units.degreesToRadians(74);
        public static final double armStowAngle = Units.degreesToRadians(-21);
        public static final double armPreAmpAngle = Units.degreesToRadians(80);
        public static final double armAmpAngle = Units.degreesToRadians(110);
        public static final double armPostAmpAngle = Units.degreesToRadians(65);

        public static final Transform2d robotToArm = new Transform2d(Units.inchesToMeters(12.0), 0.0, new Rotation2d());
        public static final double ArmHeight = Units.inchesToMeters(12);

        // For simulation.
        public static final double simArmMOI = 0.379; // kgMetersSquared
        public static final double simArmCGLength = Units.inchesToMeters(6.5); // m
        public static final double simRollerMOI = 0.003; // kgMetersSquared
        public static final double simSHooterMOI = 0.003; // kgMetersSquared
        public static final double WheelRadius = Units.inchesToMeters(2);

    }

    public static final class ShotCalc {

        public static final Pose2d targetpose = new Pose2d(16.5, 5.5, new Rotation2d(0));
        public static final double kAccelCompFactor = 0.01; // Factor for Compensating for Robot Acceleration when Shooting on the Move
        public static final InterpolatingDoubleTreeMap Velocity;
        static {
            Velocity = new InterpolatingDoubleTreeMap();
            Velocity.put(0.0, 200.0);
            Velocity.put(2.0, 250.0);
            Velocity.put(3.0, 300.0);
            Velocity.put(4.0, 350.0);
        }
        public static final InterpolatingDoubleTreeMap Angle;
        static {
            Angle = new InterpolatingDoubleTreeMap();
            Angle.put(0.0, Units.degreesToRadians(70));

            Angle.put(1.0, Units.degreesToRadians(55));
            Angle.put(2.0, Units.degreesToRadians(45));
            Angle.put(3.0, Units.degreesToRadians(35));
            Angle.put(4.0, Units.degreesToRadians(30));
            Angle.put(5.0, Units.degreesToRadians(26));
        }
    }

    public static final class Viz {
        public static final double xOffset = Units.inchesToMeters(30.0);

        public static final double intakeX = xOffset + Units.inchesToMeters(28.25);
        public static final double intakeY = Units.inchesToMeters(5);

        public static final double ArmArmPivotX = xOffset + Units.inchesToMeters(4.0);
        public static final double ArmArmPivotY = Units.inchesToMeters(12.0);
        public static final double ArmArmLength = Units.inchesToMeters(20.0);
        public static final double ArmRollerX = ArmArmLength - Units.inchesToMeters(4.0);
        public static final double ArmRollerY = Units.inchesToMeters(2);
        public static final double ArmShooterX = ArmArmLength - Units.inchesToMeters(1.0);
        public static final double ArmShooterY = Units.inchesToMeters(2);

        public static final double climberBaseX = xOffset + Units.inchesToMeters(14.0);
        public static final double climberBaseY = Units.inchesToMeters(3.0);
        public static final Rotation2d climberAngle = Rotation2d.fromDegrees(90.0);
        public static final double climberBaseLength = Units.inchesToMeters(15.0);
        public static final double climberCarriageLength = Units.inchesToMeters(6.0);

        public static final double angularVelocityScalar = 0.01;
    }

    public static final class Viz3d {
        public static double stage1Height = Units.inchesToMeters(26.0);
        public static final Pose3d intakePivotBase = new Pose3d(Units.inchesToMeters(-12.5), 0.0,
                Units.inchesToMeters(11.0), new Rotation3d());
        public static final Pose3d ArmBase = new Pose3d(
                Units.inchesToMeters(3.5),
                0,
                Units.inchesToMeters(4.0),
                new Rotation3d(0, 0, 0));
        public static final Transform3d elevatorCarriageToLauncherArmPivot = new Transform3d(0, 0,
                Units.inchesToMeters(16.0), new Rotation3d());
    }

    public static final class Intake {
        public static final int beamBreakPort = 0;

        public static final double rollerIntakePower = 1.0;
        public static final double reverseRollerIntakePower = -0.50;
        public static final double rollerSlowPower = 0.25;
        public static final double rollerStallSpeed = Math.PI * Math.PI * (1.0 / 32.0);
        public static final double rollerStallTime = 0.1;
        public static final int intakeBeamBreakInputChannel = 0;

        public static final CANDeviceID rollerMotorID = new CANDeviceID(21, kRioName);

        public static final MechanismRatio rollerMotorRatio = new MechanismRatio(1, 3);
        public static final boolean rollerMotorInvert = false;
        public static final SimpleMotorFeedforward rollerFeedforward = new SimpleMotorFeedforward(0, 0.019);
        public static final PIDConfig rollerPIDConfig = new PIDConfig(2.0, 0, 0);
        public static final int rollerVelocitySlot = 0;

        public static final double intakeRollerVelocity = 80; // rad/s
        public static final double outtakeRollerVelocity = -40; // rad/s
        public static final double holdRollerVelocity = 20; // rad/s

        // For simulation.
        public static final double simRollerMOI = 0.01; // kgMetersSquared
    }

    public static final class Climber {
        public static final CANDeviceID rightID = new CANDeviceID(23, kRioName);
        public static final CANDeviceID leftID = new CANDeviceID(22, kRioName);
        public static final double StatorLimit = 80.0;
        public static final double SupplyLimit = 40.0;
        public static final double sprocketPitchDiameter = Units.inchesToMeters(2);
        public static final MechanismRatio motorRatio = new MechanismRatio(
            1, 20, Math.PI * sprocketPitchDiameter);
        public static final boolean rightInvert = false;
        public static final boolean leftInvert = true;
        public static final int motorPositionSlot = 0;
        public static final PIDConfig motorPIDConfig = new PIDConfig(2.0, 0.0, 0.0);
        public static final double maxVelocity = 0.4; // m/s
        public static final double maxAcceleration = 30.0; // m/s^2
        public static final double maxJerk = 0.0; // m/s^3 (0 disables jerk limit)

        // TODO: use real numbers
        public static final double minHeight = Units.inchesToMeters(-25.0); // m
        // public static final double powerCutoffHeight = Units.inchesToMeters(0.1); //
        // m
        public static final double maxHeight = Units.inchesToMeters(25.0); // m
        public static final double stowHeight = Units.inchesToMeters(0); // m
        public static final double upperStowHeight = Units.inchesToMeters(-8.1); // m //-1.3141 -8.2623
        public static final double climbHeight = Units.inchesToMeters(1.1); // m
        public static final double stowTolerance = Units.inchesToMeters(0.01); // m

        // For simulation.
        public static final double simCarriageMass = 69.0; // kg

        // TODO: find real values
        public static final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0.35, 0.15, 15.8);
    }

}