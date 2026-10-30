package org.firstinspires.ftc.teamcode.vcsc.subsystems;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.vcsc.external.goBildaPrism.Color;
import org.firstinspires.ftc.teamcode.vcsc.external.goBildaPrism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.vcsc.external.goBildaPrism.PrismAnimations;
import org.firstinspires.ftc.teamcode.vcsc.subsystems.alliance.AllianceColor;
import org.firstinspires.ftc.teamcode.vcsc.subsystems.posesystems.PoseConstants;

public class Shared {
    public static double rpmToTPS(double rpm) {
        final double TICKS_PER_REVOLUTION = 28.0;
        return (rpm * TICKS_PER_REVOLUTION) / 60;
    }

    // Auto and Tele Start
    public static void SnakeColors(GoBildaPrismDriver ledStick, AllianceColor allianceColor) {
        PrismAnimations.Snakes snakes = new PrismAnimations.Snakes(Color.GREEN);
        if (allianceColor == AllianceColor.BLUE) {
            snakes.setColors(Color.BLUE);
        } else {
            snakes.setColors(Color.RED);
        }
        snakes.setBrightness(75);
        snakes.setSnakeLength(20);
        snakes.setSpeed(0.1f);
        ledStick.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, snakes);
    }

    // Tele end
    public static void SnakeColors(GoBildaPrismDriver ledStick) {
        PrismAnimations.Snakes snakes = new PrismAnimations.Snakes(Color.GREEN);
        snakes.setBrightness(75);
        snakes.setSnakeLength(20);
        snakes.setSpeed(0.1f);
        ledStick.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, snakes);
    }

    public static Pose MirrorPose(Pose pose) {
        return PoseConstants.p.of(144 - pose.x(), 144 - pose.y(), pose.heading() - 180);
    }
}
