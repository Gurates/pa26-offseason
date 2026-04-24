package frc.robot.constants;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public final class HoodConstants {
    private HoodConstants() {
    }

    public static final int HOOD_MOTOR_ID = 99;
    
    public static final double HOOD_GEAR_RATIO       = 20.0;
    public static final double HOOD_OPENED_DEGREES   = 0.0;
    public static final double HOOD_CLOSED_DEGREES   = 45.0;
    
    public static final double HOOD_kP = 0.004;
    public static final double HOOD_kI = 0.0;
    public static final double HOOD_kD = 0.01;
    
    public static final double HOOD_POSITION_TOLERANCE        = 1.0;
    public static final double HOOD_SETTLED_VELOCITY_THRESHOLD = 5.0;
    
    public static final double MIN_SAFE_DISTANCE = 1.5;
    public static final double MAX_SAFE_DISTANCE = 5.0;
    
    public static final InterpolatingDoubleTreeMap DISTANCE_TO_HOOD_ANGLE_MAP = new InterpolatingDoubleTreeMap();
    
    static {
        DISTANCE_TO_HOOD_ANGLE_MAP.put(1.5, 45.0);
        DISTANCE_TO_HOOD_ANGLE_MAP.put(2.5, 38.0);
        DISTANCE_TO_HOOD_ANGLE_MAP.put(3.0, 33.0);
        DISTANCE_TO_HOOD_ANGLE_MAP.put(3.5, 28.0);
        DISTANCE_TO_HOOD_ANGLE_MAP.put(4.0, 23.0);
        DISTANCE_TO_HOOD_ANGLE_MAP.put(4.5, 18.0);
        DISTANCE_TO_HOOD_ANGLE_MAP.put(5.0, 14.0);
    }

}