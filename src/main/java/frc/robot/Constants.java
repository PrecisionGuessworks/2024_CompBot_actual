package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.util.Units;
import frc.robot.motorcontrol.MechanismRatio;
import frc.robot.motorcontrol.devices.CANDeviceID;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Constants {
    public static final String kCanivoreName = "canivore";
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

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double kSlipCurrentA = 300.0;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    private static final double kSpeedAt12VoltsMps = 5.0292;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 0;

    private static final double kDriveGearRatio = 6.75;
    private static final double kSteerGearRatio = 21.4285714286;
    private static final double kWheelRadiusInches = 2; //may mean diameter

    private static final boolean kSteerMotorReversed = true;
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

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
    private static final double kFrontLeftEncoderOffset = -0.207275390625;

    private static final double kFrontLeftXPosInches = 10.375;
    private static final double kFrontLeftYPosInches = 10.375;

    // Front Right
    private static final int kFrontRightDriveMotorId = 6;
    private static final int kFrontRightSteerMotorId = 7;
    private static final int kFrontRightEncoderId = 8;
    private static final double kFrontRightEncoderOffset = -0.444091796875;

    private static final double kFrontRightXPosInches = 10.375;
    private static final double kFrontRightYPosInches = -10.375;

    // Back Left
    private static final int kBackLeftDriveMotorId = 12;
    private static final int kBackLeftSteerMotorId = 13;
    private static final int kBackLeftEncoderId = 14;
    private static final double kBackLeftEncoderOffset = -0.37841796875;

    private static final double kBackLeftXPosInches = -10.375;
    private static final double kBackLeftYPosInches = 10.375;

    // Back Right
    private static final int kBackRightDriveMotorId = 9;
    private static final int kBackRightSteerMotorId = 10;
    private static final int kBackRightEncoderId = 11;
    private static final double kBackRightEncoderOffset = -0.4638671875;

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
                public static final CANDeviceID rollerMotorID = new CANDeviceID(21, kCanivoreName);
                public static final MechanismRatio rollerMotorRatio = new MechanismRatio(0,0,0);
                public static final boolean rollerMotorInverted = false;

                public static final double rollerIntakePower = 0.50;
                public static final double rollerSlowPower = 0.25;

                public static final double rollerStallSpeed = Math.PI * (1.0 / 8.0);
                public static final double rollerStallTime = 0.1;

        }
        
    }
}


