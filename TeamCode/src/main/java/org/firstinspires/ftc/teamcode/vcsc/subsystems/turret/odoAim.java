package org.firstinspires.ftc.teamcode.vcsc.subsystems.turret;

import android.hardware.HardwareBuffer;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class odoAim {
    private Servo turretHeadingServo;
    private Follower follower;
    private Limelight3A limelight;

    private Pose targetPose;
    private Pose audienceTargetPose;
    private Pose scoringTargetPose;

    private double turretPosition;

    private double targetTurretHeading;

    // the PedroPathing coordinates of each hive
    final Pose REDTARGETAUDIENCE = new Pose(0, 0);
    final Pose REDTARGETSCORING = new Pose(0,0);
    final Pose BLUETARGETAUDIENCE = new Pose(0,0);
    final Pose BLUETARGETSCORING = new Pose(0,0);

    // odoAim initialization
    public odoAim(HardwareMap hardwareMap, Follower follower, boolean isRed) {

        // changes tracked target depending on if we are on blue or red
        this.audienceTargetPose = isRed ? REDTARGETAUDIENCE : BLUETARGETAUDIENCE;
        this.scoringTargetPose = isRed ? REDTARGETSCORING : BLUETARGETSCORING;

    }

    // odoAim looped method
    public void updateTurretHeading() {
        // changes the target to either the scoring side or the audience side depending on what side our robot is on
        this.targetPose = follower.pose().y() > 72 ? scoringTargetPose : audienceTargetPose;

        // converts PedroPathing coordinates into robot relative coordinates
        double robotRelativeTargetX = targetPose.x() - follower.pose().x();
        double robotRelativeTargetY = targetPose.y() - follower.pose().y();

        // calculates the angle needed to rotate the turret to those coordinates
        double robotRelativeTargetAngle = Math.atan2(robotRelativeTargetY, robotRelativeTargetX);

        /*
        determines how far the turret needs to rotate to reach the target angle by subtracting how much
        the robot has currently rotated and normalizing the radian measurement between pi and -pi (180 and -180 degrees)
         */
        this.targetTurretHeading = AngleUnit.normalizeRadians(robotRelativeTargetAngle - follower.pose().heading());
    }
}
