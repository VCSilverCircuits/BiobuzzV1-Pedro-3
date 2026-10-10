package org.firstinspires.ftc.teamcode.vcsc.subsystems.posesystems;

import com.pedropathing.math.Pose;

public class PoseStorage {
    /*
    * At the start of auto scripts set 'autoEnd' to null and 'turretRadianAutoEnd' to 0.0
    * Set this as the end position at the end of auto
    * */
    // End of auto robot position
    public static Pose autoEnd = null;
    // End of auto turret rotation
    public static double turretRadiansAutoEnd = 0.0;
}