package frc.robot.subsystems.CANdle;

public enum Colors {
    RED(255, 0, 0),
    GREEN(0,255,0),
    BLUE(0,0,255),
    ORANGE(255, 165, 0),
    WHITE(255,255,255),
    OFF(0, 0, 0);


    Colors color;
    private final int[] rgb = new int[3];
    private Colors(int r, int g, int b){
        rgb[0] = r;
        rgb[1] = g;
        rgb[2] = b;
    }

    public int[] getColorVal() {
        return rgb;
    }
    public int getRedVal(){
        return rgb[0];
    }
    public int getGreenVal(){
        return rgb[1];
    }
    public int getBlueVal(){
        return rgb[2];
    }


}

