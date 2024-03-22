package frc.robot;


import java.util.ArrayList;
import java.util.HashMap;

public class ShotDistTable {
    public static final double maxShotDist = 4.0; //meters
    public static final double maxArmDist = 6.0; //meters
    public HashMap<Double, Double> shotTableAngles = new HashMap<Double,Double>();
    

    public ShotDistTable() {
        shotTableAngles.put(0.0, 94.0);
        shotTableAngles.put(4.0, 45.0);
    }

    public double calculate(double distance) {
        Object[] shotKeys =  shotTableAngles.keySet().toArray();
        double lowBound = (double)shotKeys[0];
        double highBound = (double)shotKeys[shotKeys.length];

        for (int i = 0; i < shotTableAngles.size(); i++) {
            double currKey = (double)shotKeys[i];
            if (currKey <= distance && currKey >= lowBound) {
                lowBound = currKey;

            }
            
            if (currKey >= distance && currKey <=highBound) {
                highBound = currKey;
            }

        }
        double angle = this.interpolate(lowBound, highBound, shotTableAngles.get(lowBound), shotTableAngles.get(highBound), distance);

        return angle;


    }

    private double interpolate(double x0, double x1, double y0, double y1, double x) {
        double y = ((y0 * ((x1-x)/(x1-x0))) + (y1 * ((x-x0)/(x1-x0))));

        return y;
    }
    
}
