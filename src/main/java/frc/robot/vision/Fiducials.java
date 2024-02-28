package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class Fiducials {
    public class AprilTags {
        public class RedSpeakerTag {
            public static int ID = 4;

            public static Pose3d pose = new Pose3d(
                Units.inchesToMeters(652.73),
                Units.inchesToMeters(218.42),
                Units.inchesToMeters(57.13),
                new Rotation3d(0.0, 0.0, Math.toRadians(180.0)));
        }

        public class BlueSpeakerTag {
            public int ID = 7;

            public static Pose3d pose = new Pose3d(
                Units.inchesToMeters(-1.50),
                Units.inchesToMeters(218.42),
                Units.inchesToMeters(57.13),
                new Rotation3d(0.0, 0.0, Math.toRadians(0.0)));
        }
    }
    
}
