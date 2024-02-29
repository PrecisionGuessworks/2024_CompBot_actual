package frc.robot.vision;

import java.lang.reflect.Array;
import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class Fiducials {
    
    
    public class AprilTags {
        public static final PresAprilTag[] aprilTagFiducials =
      new PresAprilTag[] {
        new PresAprilTag(
           
            1,
            new Pose3d(
                Units.inchesToMeters(593.68),
                Units.inchesToMeters(9.68),
                Units.inchesToMeters(53.38),
                new Rotation3d(0.0, 0.0, Math.toRadians(120.0)))
            ),
        new PresAprilTag(
            
            2,
            new Pose3d(
                Units.inchesToMeters(637.21),
                Units.inchesToMeters(34.79),
                Units.inchesToMeters(53.38),
                new Rotation3d(0.0, 0.0, Math.toRadians(120.0)))),
        new PresAprilTag(
            
            3,
            new Pose3d(
                Units.inchesToMeters(652.73),
                Units.inchesToMeters(196.17),
                Units.inchesToMeters(57.13),
                new Rotation3d(0.0, 0.0, Math.toRadians(180.0)))),
        new PresAprilTag(
            
            4,
            new Pose3d(
                Units.inchesToMeters(652.73),
                Units.inchesToMeters(218.42),
                Units.inchesToMeters(57.13),
                new Rotation3d(0.0, 0.0, Math.toRadians(180.0)))),
        new PresAprilTag(
            
            5,
            new Pose3d(
                Units.inchesToMeters(578.77),
                Units.inchesToMeters(323.00),
                Units.inchesToMeters(53.38),
                new Rotation3d(0.0, 0.0, Math.toRadians(270.0)))),
        new PresAprilTag(
           
            6,
            new Pose3d(
                Units.inchesToMeters(72.50),
                Units.inchesToMeters(323.00),
                Units.inchesToMeters(53.38),
                new Rotation3d(0.0, 0.0, Math.toRadians(270.0)))),
        new PresAprilTag(
            
            7,
            new Pose3d(
                Units.inchesToMeters(-1.50),
                Units.inchesToMeters(218.42),
                Units.inchesToMeters(57.13),
                new Rotation3d(0.0, 0.0, Math.toRadians(0.0)))),
        new PresAprilTag(
            
            8,
            new Pose3d(
                Units.inchesToMeters(-1.50),
                Units.inchesToMeters(196.17),
                Units.inchesToMeters(57.13),
                new Rotation3d(0.0, 0.0, Math.toRadians(0.0)))),
        new PresAprilTag(
            9,
            new Pose3d(
                Units.inchesToMeters(14.02),
                Units.inchesToMeters(34.79),
                Units.inchesToMeters(53.38),
                new Rotation3d(0.0, 0.0, Math.toRadians(60.0)))),
        new PresAprilTag(
            
            10,
            new Pose3d(
                Units.inchesToMeters(57.54),
                Units.inchesToMeters(9.68),
                Units.inchesToMeters(53.38),
                new Rotation3d(0.0, 0.0, Math.toRadians(60.0)))),
        new PresAprilTag(
            
            11,
            new Pose3d(
                Units.inchesToMeters(468.69),
                Units.inchesToMeters(146.19),
                Units.inchesToMeters(52.00),
                new Rotation3d(0.0, 0.0, Math.toRadians(300.0)))),
        new PresAprilTag(
           
            12,
            new Pose3d(
                Units.inchesToMeters(468.69),
                Units.inchesToMeters(177.10),
                Units.inchesToMeters(52.00),
                new Rotation3d(0.0, 0.0, Math.toRadians(60.0)))),
        new PresAprilTag(
            
            13,
            new Pose3d(
                Units.inchesToMeters(441.74),
                Units.inchesToMeters(161.62),
                Units.inchesToMeters(52.00),
                new Rotation3d(0.0, 0.0, Math.toRadians(180.0)))),
        new PresAprilTag(
            
            14,
            new Pose3d(
                Units.inchesToMeters(209.48),
                Units.inchesToMeters(161.62),
                Units.inchesToMeters(52.00),
                new Rotation3d(0.0, 0.0, Math.toRadians(0.0)))),
        new PresAprilTag(
            
            15,
            new Pose3d(
                Units.inchesToMeters(182.73),
                Units.inchesToMeters(177.10),
                Units.inchesToMeters(52.00),
                new Rotation3d(0.0, 0.0, Math.toRadians(120.0)))),
        new PresAprilTag(
            
            16,
            new Pose3d(
                Units.inchesToMeters(182.73),
                Units.inchesToMeters(146.19),
                Units.inchesToMeters(52.00),
                new Rotation3d(0.0, 0.0, Math.toRadians(240.0)))),
      };


    
    }
}
