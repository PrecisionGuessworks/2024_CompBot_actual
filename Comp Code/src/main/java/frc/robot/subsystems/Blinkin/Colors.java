package frc.robot.subsystems.Blinkin;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotContainer;

public enum Colors {
    RED(0.61),
    GREEN(0.77),
    WHITE(0.93),
    PURPLE(0.91),
    BLUE(0.87),
    HOT_PINK (0.57),
    DARK_RED (0.59),
    RED_ORANGE (0.63),
    ORANGE (0.65),
    GOLD (0.67),
    LAWN_GREEN (0.71),
    LIME (0.73),
    BLUE_GREEN (0.79),
    AQUA (0.81),
    SKY_BLUE (0.83),
    DARK_BLUE (0.85),
    BLUE_VIOLET (0.89),
    YELLOW(0.69),
    RAINBOW_COLOR_WAVES(-0.45),
    OFF(0.99);


    Colors color;
    private final double colorVal;
    private Colors(double colorVal){
        this.colorVal = colorVal;
    }

    public double getColorVal() {
        return colorVal;
    }

    

}


