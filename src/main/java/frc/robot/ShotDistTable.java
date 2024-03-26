package frc.robot;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.TreeMap;

public class ShotDistTable {
    public static final double maxShotDist = 3.0; //meters
    public static final double maxArmDist = 4.0; //meters
    public TreeMap<Double, Double> shotTableAngles = new TreeMap<Double,Double>();
    

    public ShotDistTable() {

        shotTableAngles.put(0.0, 94.0);
        shotTableAngles.put(0.8, 74.0);
        shotTableAngles.put(2.8, 47.2);
        shotTableAngles.put(3.0, 45.0);
        
    }

    public double calculate(double distance) {
        Object[] shotKeys =  shotTableAngles.keySet().toArray();
        System.out.println(Arrays.toString(shotKeys));
        double lowBound = (double)shotKeys[0];
        double highBound = (double)shotKeys[shotKeys.length-1];

        for (int i = 0; i < shotTableAngles.size(); i++) {
            double currKey = (double)shotKeys[i];
            if (currKey <= distance && currKey >= lowBound) {
                lowBound = currKey;

            }
            
            if (currKey >= distance && currKey <=highBound) {
                highBound = currKey;
            }

        }
        double angle = interpolate(lowBound, highBound, shotTableAngles.get(lowBound), shotTableAngles.get(highBound), distance);

        return angle;


    }

    private double interpolate(double x0, double x1, double y0, double y1, double x) {
        double y = ((y0 * ((x1-x)/(x1-x0))) + (y1 * ((x-x0)/(x1-x0))));

        return y;
    }
    
}
