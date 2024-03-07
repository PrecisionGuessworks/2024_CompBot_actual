package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.CAN;
import frc.robot.motorcontrol.MechanismRatio;
import frc.robot.motorcontrol.PIDConfig;
import frc.robot.motorcontrol.devices.CANDeviceID;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Constants {
    public static final String kCanivoreName = "canivore";
    public static final String kRioName = "rio";
    public static final class Swerve {
        public class TunerConstants {
    // Both sets of gains need to be tuned to your individual robot
    // The steer motor uses MotionMagicVoltage control
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.05)
        .withKS(0).withKV(1.5).withKA(0);
    // When using closed-loop control, the drive motor uses:
    // - VelocityVoltage, if DrivetrainConstants.SupportsPro is false (default)
    // - VelocityTorqueCurrentFOC, if DrivetrainConstants.SupportsPro is true

     private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(3).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);

   // private static final Slot0Configs driveGains = new Slot0Configs().withKP(3).withKI(0).withKD(0).withKS(0.48665 / 12).withKV(2.4132 / 12).withKA(0.06921 / 12);


 // Crev Controller Constants
//  public static final double kDriveS = (0.48665 / 12.0);
//  public static final double kDriveV = (2.4132 / 12.0);
//  public static final double kDriveA = (0.06921 / 12.0);

//  public static final double kDriveP = 0.01; // 0.08;
//  public static final double kDriveI = 0.0;
//  public static final double kDriveD = 0; // 0.1;

//  public static final double kAngleP = 0.09;
//  public static final double kAngleI = 0.0;
//  public static final double kAngleD = 0.1;


    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double kSlipCurrentA = 300.0;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double kSpeedAt12VoltsMps = 5.0292;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 0;

    private static final double kDriveGearRatio = 6.75;
    private static final double kSteerGearRatio = 21.4285714286;
    private static final double kWheelRadiusInches = 2; //may mean diameter

    private static final boolean kSteerMotorReversed = true;
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;
    private static final boolean kDriveMotorReversed = true;

    private static final String kCANbusName = kCanivoreName;
    private static final int kPigeonId = 2;

    // These are only used for simulation
    private static final double kSteerInertia = 0.00001;
    private static final double kDriveInertia = 0.001;

    private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withPigeon2Id(kPigeonId)
            .withCANbusName(kCANbusName);

    private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadiusInches)
            .withSlipCurrent(kSlipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(kCoupleRatio)
            .withSteerMotorInverted(kSteerMotorReversed);

    // Front Left
    private static final int kFrontLeftDriveMotorId = 3;
    private static final int kFrontLeftSteerMotorId = 4;
    private static final int kFrontLeftEncoderId = 5;
    private static final double kFrontLeftEncoderOffset = 0.282470703125;

    private static final double kFrontLeftXPosInches = 10.375;
    private static final double kFrontLeftYPosInches = 10.375;

    // Front Right
    private static final int kFrontRightDriveMotorId = 6;
    private static final int kFrontRightSteerMotorId = 7;
    private static final int kFrontRightEncoderId = 8;
    private static final double kFrontRightEncoderOffset = 0.0986328125;

    private static final double kFrontRightXPosInches = 10.375;
    private static final double kFrontRightYPosInches = -10.375;

    // Back Left
    private static final int kBackLeftDriveMotorId = 12;
    private static final int kBackLeftSteerMotorId = 13;
    private static final int kBackLeftEncoderId = 14;
    private static final double kBackLeftEncoderOffset = 0.36279296875;

    private static final double kBackLeftXPosInches = -10.375;
    private static final double kBackLeftYPosInches = 10.375;

    // Back Right
    private static final int kBackRightDriveMotorId = 9;
    private static final int kBackRightSteerMotorId = 10;
    private static final int kBackRightEncoderId = 11;
    private static final double kBackRightEncoderOffset = 0.288818359375;

    private static final double kBackRightXPosInches = -10.375;
    private static final double kBackRightYPosInches = -10.375;

    private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
    private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
    private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
    private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);

    public static final CommandSwerveDrivetrain DriveTrain = new CommandSwerveDrivetrain(DrivetrainConstants, FrontLeft,
            FrontRight, BackLeft, BackRight);
}

    }
     public static final class Example {
        public static final CANDeviceID motorID = new CANDeviceID(0);
        public static final MechanismRatio motorRatio = new MechanismRatio(0, 0, 0);
    }

    public static final class Intake {
        public static final class Roller {
                public static final CANDeviceID rollerMotorID = new CANDeviceID(21, kRioName);
                public static final MechanismRatio rollerMotorRatio = new MechanismRatio(1,3);
                public static final boolean rollerMotorInverted = false;

                public static final double rollerIntakePower = 1.0;
                public static final double reverseRollerIntakePower = -0.50;
                public static final double rollerSlowPower = 0.25;

                public static final double rollerStallSpeed = Math.PI * Math.PI * (1.0 / 32.0);
                public static final double rollerStallTime = 0.1;

                public static final int intakeBeamBreakInputChannel = 0;

        }
        
    }

    public static final class Shooter {
        public static final class TopRoller {
                public static final CANDeviceID topRollerID = new CANDeviceID(19, kRioName);
                public static final MechanismRatio topRollerMotorRatio = new MechanismRatio(1,1);
                public static final boolean topRollerMotorInverted = true;
                public static final int topRollerMotorSlot = 0;
                public static final PIDConfig topMotorPIDConfig = new PIDConfig(2.0, 0.0, 0.0);

                public static final SimpleMotorFeedforward topRollerFeedforward =
        new SimpleMotorFeedforward(0, 0.019);

                public static final double topIntakePower = -0.4;
                public static final double topEjectPower = 0.7;
        }

        public static final class BottomRoller {
                public static final CANDeviceID bottomRollerID = new CANDeviceID(20, kRioName);
                public static final MechanismRatio bottomRollerMotorRatio = new MechanismRatio(1,1);
                public static final boolean bottomRollerMotorInverted = true;
                public static final int bottomRollerMotorSlot = 0;
                public static final PIDConfig bottomMotorPIDConfig = new PIDConfig(2.0, 0.0, 0.0);

                public static final SimpleMotorFeedforward bottomRollerFeedforward =
        new SimpleMotorFeedforward(0, 0.019);

                public static final double bottomIntakePower = -0.4;
                public static final double bottomEjectPower = 0.7;
        }

        public static final class Conveyer {
                public static final CANDeviceID conveyerID = new CANDeviceID(17, kRioName);
                public static final MechanismRatio conveyerMotorRatio = new MechanismRatio(1,4);
                public static final boolean conveyerMotorInverted = true;
                public static final int conveyerRollerMotorSlot = 0;
                public static final PIDConfig conveyerMotorPIDConfig = new PIDConfig(2.0, 0.0, 0.0);
                public static final SimpleMotorFeedforward feedFeedforward =
        new SimpleMotorFeedforward(0, 0.019);
                public static final double conveyerIntakePower = -0.22;
        }

        public static final class Amp {
                public static final CANDeviceID ampID = new CANDeviceID(18, kRioName);
                public static final MechanismRatio ampMotorRatio = new MechanismRatio(1,4);
                public static final boolean ampMotorInverted = false;
                public static final int ampRollerMotorSlot = 0;
                public static final PIDConfig ampMotorPIDConfig = new PIDConfig(2.0, 0.0, 0.0);
                public static final SimpleMotorFeedforward ampFeedforward =
        new SimpleMotorFeedforward(0, 0.019);
                public static final double ampIntakePower = -0.22;

        }
       
        public static final double launchVelocity = 400.0; // rads/s
        public static final double launchVelocityTolerance = 20.0; // rads/s
        public static final double ScoreAmpPower = 0.5;
        public static final double AmpVelocityTolerance = 10.0; // rads/s

        public static final double intakeFeedVelocity = 80; // rad/s
        public static final double scoreAmpFeedVelocity = 80; // rad/s
        public static final double scoreSpeakerFeedVelocity = 160; // rad/s
        
        
    }

    public static final class Arm {
        public static final class RightPivot {
                public static final CANDeviceID rightPivotID = new CANDeviceID(16, kRioName);
                public static final int rightPivotMotorSlot = 0;
                public static final PIDConfig rightPivotPIDConfig = new PIDConfig(0.7, 0.01, 0.03);
                // TODO: Check ratio
                public static final MechanismRatio rightPivotRatio = new MechanismRatio(1, 75);
                
                public static final boolean rightPivotInvert = false;
                public static final double rightPivotAccelerationConstraint = 1.5; // rad/s
                public static final double rightPivotVelocityConstraint = 2.5; // rad/s

                public static final Constraints rightPivotTrapConstraints = new Constraints(rightPivotVelocityConstraint, rightPivotAccelerationConstraint);
                //public static final double rightPivotAccelerationConstraintDown = 0.05; // rad/s
               // public static final double rightPivotVelocityConstraintDown = 0.05; // rad/s

                //public static final Constraints rightPivotTrapConstraintsDown = new Constraints(rightPivotVelocityConstraintDown, rightPivotAccelerationConstraintDown);

                public static final ArmFeedforward rightFeedForward = new ArmFeedforward(0.02, 0.92, 1.35, 0.06);
                
        }
        
        public static final class LeftPivot {
                public static final CANDeviceID leftPivotID = new CANDeviceID(15, kRioName);
                public static final int leftPivotMotorSlot = 0;
                public static final PIDConfig leftPivotPIDConfig = new PIDConfig(0.7, 0.01, 0.03);
                // TODO: Check ratio
                public static final MechanismRatio leftPivotRatio = new MechanismRatio(1, 75);
                public static final boolean leftPivotInvert = true;

                public static final double leftPivotAccelerationConstraint = 1.0; // rad/s
                public static final double leftPivotVelocityConstraint = 1.0; // rad/s

                public static final Constraints leftPivotTrapConstraints = new Constraints(leftPivotVelocityConstraint, leftPivotAccelerationConstraint);

               // public static final double leftPivotAccelerationConstraintDown = 0.05; // rad/s
                //public static final double leftPivotVelocityConstraintDown = 0.05; // rad/s

              //  public static final Constraints leftPivotTrapConstraintsDown = new Constraints(leftPivotVelocityConstraintDown, leftPivotAccelerationConstraintDown);

                public static final ArmFeedforward leftFeedForward = new ArmFeedforward(0.02, 0.92, 1.35, 0.06);
             
        }

        public static final class ArmEnconder {
                public static final CANDeviceID encoderID = new CANDeviceID(24, kRioName);
                public static final MechanismRatio armRatio = new MechanismRatio(1,1);
                public static final MechanismRatio encoderToMotorRatio = new MechanismRatio(1,75);
                
        }

        public static final double minAngle = Units.degreesToRadians(0);
        public static final double maxAngle = Units.degreesToRadians(110);
        public static final double startingAngle = minAngle;
        public static final double intakeAngle = Units.degreesToRadians(3);
        public static final double intakeAngleTolerance = Units.degreesToRadians(8);

        public static final double launchAngle = Units.degreesToRadians(74);
        public static final double ampPreAngle = Units.degreesToRadians(80);
        public static final double launchAngleTolerance = Units.degreesToRadians(3);
        public static final double scoreAmpArmAngle = Units.degreesToRadians(110); // rads
        public static final double moveAmpArmAngle = Units.degreesToRadians(65); 
        public static final double scoreAmpArmAngleTolerance = Units.degreesToRadians(5); // rads
        public static final double midpointAngle = Units.degreesToRadians(75); 
        public static final double midpointAngleTolerance = Units.degreesToRadians(5);
        public static final double ShootTimeout = 8;
        public static final double AmpTimeout = 15;
        public static final double AmpTimeoutMid = 15;
        public static final double eject = Units.degreesToRadians(30);
        public static final double ejectAngleTolerance = Units.degreesToRadians(3);

        //public static final TrapezoidProfile.State m_goal = new TrapezoidProfile.State(100, 0);


    }

    public static final class Climber {
        public static final class rightClimber {
                public static final CANDeviceID rightClimberID = new CANDeviceID(23, kRioName);
                public static final int rightClimberMotorSlot = 0;
                public static final PIDConfig rightClimberPIDConfig = new PIDConfig(2.0, 0.0, 0.0);
                // TODO: Check ratio
                public static final MechanismRatio rightClimberRatio = new MechanismRatio(1, 20);
                public static final boolean rightClimberInvert = false;

                public static final ElevatorFeedforward rightFF = new ElevatorFeedforward(0, 0.01, 0.02, 0.012);

        }

        public static final class leftClimber {
                public static final CANDeviceID leftClimberID = new CANDeviceID(22, kRioName);
                public static final int leftClimberMotorSlot = 0;
                public static final PIDConfig leftClimberPIDConfig = new PIDConfig(2.0, 0.0, 0.0);
                // TODO: Check ratio
                public static final MechanismRatio leftClimberRatio = new MechanismRatio(1, 20);
                public static final boolean leftClimberInvert = true;

                public static final ElevatorFeedforward leftFF = new ElevatorFeedforward(0, 0.01, 0.02, 0.012);

        }

        public static final double maxPosition = 0;
        public static final double minPosition = -32.0;
        public static final double maxSpeed = 0.4; //speed gain
    }
}