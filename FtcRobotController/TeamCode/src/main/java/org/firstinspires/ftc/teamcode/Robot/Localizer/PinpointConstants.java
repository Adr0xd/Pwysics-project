package org.firstinspires.ftc.teamcode.Robot.Localizer;



import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This is the PinpointConstants class. It holds many constants and parameters for the Pinpoint Localizer.
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 12/24/2024
 */

public class PinpointConstants {

    /** The Y Offset of the Forward Encoder (Deadwheel) from the center of the robot
     * @value 1 */
    public static double forwardY = 7.2;

    /** The X Offset of the Strafe Encoder (Deadwheel) from the center of the robot
     * @value -2.5 */
    public static double strafeX = 11.5;

    public static DistanceUnit distanceUnit = DistanceUnit.CM;
    public static String hardwareMapName = "odo";
    public static boolean useYawScalar = false;
    public static double yawScalar = 1.0;
    public static boolean useCustomEncoderResolution = true;
    public static GoBildaPinpointDriver.GoBildaOdometryPods encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public static double customEncoderResolution = 4096/(35*Math.PI);
    public static GoBildaPinpointDriver.EncoderDirection forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public static GoBildaPinpointDriver.EncoderDirection strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
}