package org.firstinspires.ftc.teamcode.vcsc.subsystems.turret;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class FlywheelConstants {
    public static double p = 39.5;
    public static double i = 0;
    public static double d = 34;
    public static double f = 23;
    public static double flywheelRPM = 3000;
    public static double hoodAnglePos = 0;

    private static boolean isAuto = false;

    public static void loop() {
        // update pidf and flywheel depending on robot pos and best place to put it
    }

    public static void setIsAuto(boolean isAuto) {
        FlywheelConstants.isAuto = isAuto;
    }
}
