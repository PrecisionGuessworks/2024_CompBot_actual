package frc.robot;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.TreeMap;

public class ShotDistTable {
    public double maxShotDist = 3.0; //meters
    public double maxArmDist = 3.0; //meters
    public TreeMap<Double, Double> shotTableAngles = new TreeMap<Double,Double>();
    

    public ShotDistTable() {

        shotTableAngles.put(0.0, 80.0);
        shotTableAngles.put(0.6, 74.0);
        shotTableAngles.put(0.7, 73.0);
        shotTableAngles.put(0.9, 70.0);
        shotTableAngles.put(1.2, 68.0);
        shotTableAngles.put(1.3, 67.0);
        shotTableAngles.put(1.4, 65.7);
        shotTableAngles.put(1.5, 64.7);
        shotTableAngles.put(1.6, 63.7);
        shotTableAngles.put(1.7, 62.5);
        shotTableAngles.put(1.8, 62.0);
        shotTableAngles.put(1.9, 61.5);
        shotTableAngles.put(2.0, 60.5);
        shotTableAngles.put(2.1, 59.9);
        shotTableAngles.put(2.2, 59.5);
        shotTableAngles.put(2.3, 59.0);
        shotTableAngles.put(2.4, 57.6);
        shotTableAngles.put(2.5, 57.4);
        shotTableAngles.put(2.6, 57.2);
        shotTableAngles.put(2.7, 55.7);
        shotTableAngles.put(2.8, 55.4);
        shotTableAngles.put(2.9, 55.2);
        shotTableAngles.put(3.0, 55.0);
        shotTableAngles.put(3.2, 54.9);
        shotTableAngles.put(3.4, 54.8);
        shotTableAngles.put(3.6, 54.7);
        shotTableAngles.put(3.8, 54.7);
        shotTableAngles.put(4.0, 30.0);
        
    }

    public double calculate(double distance) {
        Object[] shotKeys =  shotTableAngles.keySet().toArray();
        //System.out.println(Arrays.toString(shotKeys));
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