package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldZones {


    public record Zone(String name, double xMin, double xMax, double yMin, double yMax) {
        public boolean contains(Pose2d pose) {
            double x = pose.getX();
            double y = pose.getY();
            return x >= xMin && x <= xMax && y >= yMin && y <= yMax;
        }
    }


    private static final Zone BLUE_ZONE_1 = new Zone(
        "Blue Shooting Zone 1",
        1.0, 4.0,
        2.0, 6.0
    );

    private static final Zone BLUE_ZONE_2 = new Zone(
        "Blue Shooting Zone 2",
        4.0, 7.0,
        2.0, 6.0
    );


    private static final Zone RED_ZONE_1 = new Zone(
        "Red Shooting Zone 1",
        12.5, 15.5,
        2.0, 6.0
    );

    private static final Zone RED_ZONE_2 = new Zone(
        "Red Shooting Zone 2",
        9.5, 12.5,
        2.0, 6.0
    );

    private static boolean inZone1 = false;
    private static boolean inZone2 = false;


    public static void update(Pose2d robotPose) {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        Zone zone1 = (alliance == Alliance.Blue) ? BLUE_ZONE_1 : RED_ZONE_1;
        Zone zone2 = (alliance == Alliance.Blue) ? BLUE_ZONE_2 : RED_ZONE_2;

        inZone1 = zone1.contains(robotPose);
        inZone2 = zone2.contains(robotPose);

        SmartDashboard.putBoolean("FieldZones/Zone 1 (" + zone1.name() + ")", inZone1);
        SmartDashboard.putBoolean("FieldZones/Zone 2 (" + zone2.name() + ")", inZone2);
        SmartDashboard.putBoolean("FieldZones/In Any Zone", isInAnyZone());
        SmartDashboard.putString("FieldZones/Alliance", alliance.toString());
        SmartDashboard.putNumber("FieldZones/Robot X", robotPose.getX());
        SmartDashboard.putNumber("FieldZones/Robot Y", robotPose.getY());
    }
    public static boolean isInZone1() { return inZone1; }
    public static boolean isInZone2() { return inZone2; }
    public static boolean isInAnyZone() { return inZone1 || inZone2; }
}