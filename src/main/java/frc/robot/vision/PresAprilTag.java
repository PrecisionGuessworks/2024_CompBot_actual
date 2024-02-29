package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;

public class PresAprilTag {
    public static int ID;
    private static Pose3d Pose;

    public PresAprilTag(int id, Pose3d pose) {
        ID = id;
        Pose = pose;
    }

    public Pose3d getPose() {
        return Pose;
    }

    public int getID() {
        return ID;
    }
    
}
