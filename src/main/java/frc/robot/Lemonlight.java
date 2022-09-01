package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Lemonlight {

    static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    static NetworkTableEntry tx = table.getEntry("tx");
    static NetworkTableEntry ty = table.getEntry("ty");
    static NetworkTableEntry ta = table.getEntry("ta");
    private static NetworkTable m_table;
    private String m_tableName = "limelight";

    private Constants constants = new Constants();

    public Lemonlight() {
        m_table = NetworkTableInstance.getDefault().getTable(m_tableName);
    }

    /**
     * read values periodically
     */
    public static void lemonLightPeriodic() {
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);

    }

    /**
     * @return vertical offset to object from limeLight
     */
    public static double getVertOffset() {
        NetworkTableEntry ty = m_table.getEntry("ty");
        double a = ty.getDouble(0.0);
        return a;
    }

    /**
     * @return horizontal offset to an object from limelight
     */
    public static double getHorizontalOffset() {
        NetworkTableEntry tx = m_table.getEntry("tx");
        double b = tx.getDouble(0.0);
        return b;
    }

    /**
     * @return area of object on the limelight
     */

    public static double getArea() {
        NetworkTableEntry ta = m_table.getEntry("ta");
        double c = ta.getDouble(0.0);
        return c;
    }

    /**
     * @returns if liemlight detects a reflective tape
     */
    public static double validTarget() {
        NetworkTableEntry tv = m_table.getEntry("tv");
        double b = tv.getDouble(0.0);
        return b;
    }

    public static double GetDegreeOffset(double distance, double heightOffset, double currentAngle) {
        return Math.atan(heightOffset / distance) / Math.PI * 180 - currentAngle;
    }

    public double distanceGrab() {
        // target height - camera height
        double distance = 0;
        double heightOffset = constants.targetHeight - constants.mountedHeight;
        double staticDistance = 125;
        double readAngle = -3.09;

        double offset = GetDegreeOffset(staticDistance, heightOffset, readAngle);

        double angle = Lemonlight.getVertOffset();// this in degrees

        distance = (heightOffset) / ((Math.tan((angle + offset) / 180 * Math.PI)));
        return distance + 24;
    }

    public double getVelocity() {
        double distance = distanceGrab();// inches
        double heightFinal = constants.targetHeight;// inches
        double heightInitial = constants.mountedHeight;// inches
        double gravity = -4.9 * 3.28 * 12; // inches
        double toDeg = Math.PI / 180;
        double theta = constants.theta * toDeg;
        double delta = heightFinal - heightInitial; // inches
        double sqrt = (delta - (Math.tan(theta) * distance)) / (gravity);
        double denom = Math.cos(theta) * Math.sqrt(sqrt);
        double velocity = distance / denom;

        return velocity; // inches
    }

    public double getAngularV() {
        double linearV = getVelocity();
        double angularV = 0;
        double radius = 2; // inches
        angularV = linearV / radius; // rad/sec
        return angularV;
    }

    /**
     * 
     * 
     * @return returns an RPM by converting Angular Velocity to it
     */
    public double VtoRPM() {
        double RPM = 0;
        double angularV = getAngularV();
        RPM = (60 * angularV) / (2 * Math.PI);
        return RPM;
    }

}
