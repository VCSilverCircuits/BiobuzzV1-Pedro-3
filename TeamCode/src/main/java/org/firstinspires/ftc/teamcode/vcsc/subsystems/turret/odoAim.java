package org.firstinspires.ftc.teamcode.vcsc.subsystems.turret;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.vcsc.subsystems.alliance.AllianceColor;

// NOTE: I wrote most of this at 1AM, so the names of some variables may need to be changed to make more sense

public class odoAim {
    private Servo turretHeadingRight;
    private Servo turretHeadingLeft;
    private Follower follower;
    private Limelight3A limelight;

    private Pose targetPose;
    private Pose audienceTargetPose;
    private Pose scoringTargetPose;

    private double targetTurretHeading;

    // TODO: set the maximum and minimum rotation range of the servos based on how they were configured in the servo programmer
    private final double MIN_SERVO_RADIANS = Math.toRadians(-250); // current range of the servos (NOT THE TURRET) are -175 to 175 degrees
    private final double MAX_SERVO_RADIANS = Math.toRadians(250);

    private final double TURRET_GEAR_TO_SERVO_RATIO = 1/1; // TODO: replace 1/1 with (turret gear teeth) / (servo gear teeth)

    private boolean isManualMode; // TODO: add a manual mode toggle bound to second controller for backup if automatic tracking fails

    // the PedroPathing coordinates of each hive
    final Pose REDTARGETAUDIENCE = new Pose(58, 54);
    final Pose REDTARGETSCORING = new Pose(58,90);
    final Pose BLUETARGETAUDIENCE = new Pose(86,54);
    final Pose BLUETARGETSCORING = new Pose(86,90);

    // odoAim initialization
    public odoAim(HardwareMap hardwareMap, Follower follower, AllianceColor allianceColor) {

        // changes tracked target depending on if we are on blue or red
        this.audienceTargetPose = allianceColor == allianceColor.RED ? REDTARGETAUDIENCE : BLUETARGETAUDIENCE;
        this.scoringTargetPose = allianceColor == allianceColor.RED ? REDTARGETSCORING : BLUETARGETSCORING;

        // hardware maps the turret heading servos
        turretHeadingRight = hardwareMap.get(Servo.class, "rightTurret");
        turretHeadingLeft = hardwareMap.get(Servo.class, "leftTurret");

        // passes the follower into the instance of this class
        this.follower = follower;
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
        the robot has currently rotated
         */
        this.targetTurretHeading = robotRelativeTargetAngle - follower.pose().heading();

        /* gets the normalized target turret heading and the absolute value of that to determine if
        the target is within the set range that the hardware can safely reach
         */
        double normalizedTargetServoHeading = AngleUnit.normalizeRadians(targetTurretHeading / TURRET_GEAR_TO_SERVO_RATIO);
        double absoluteNormalizedTargetServoHeading = Math.abs(normalizedTargetServoHeading);

        // sets the servo rotation to the target angle and adjusts it to fit within the range if necessary
        if (!isManualMode) {
            if (absoluteNormalizedTargetServoHeading <= MAX_SERVO_RADIANS - Math.toRadians(5)) {
                turretHeadingRight.setPosition(normalizeToServoRotations(targetTurretHeading));
                turretHeadingLeft.setPosition(normalizeToServoRotations(targetTurretHeading));

            } else if (normalizedTargetServoHeading >= MAX_SERVO_RADIANS - Math.toRadians(5)) {
                turretHeadingRight.setPosition(normalizeToServoRotations(MAX_SERVO_RADIANS - Math.toRadians(5)));
                turretHeadingLeft.setPosition(normalizeToServoRotations(MAX_SERVO_RADIANS - 5));

            } else if (normalizedTargetServoHeading >= MIN_SERVO_RADIANS + Math.toRadians(5)) {
                turretHeadingRight.setPosition(normalizeToServoRotations(MIN_SERVO_RADIANS + Math.toRadians(5)));
                turretHeadingLeft.setPosition(normalizeToServoRotations(MIN_SERVO_RADIANS + Math.toRadians(5)));
            }
        }
    }
    public void auto() {
        this.isManualMode = true;
    }
    public void manual() {
        this.isManualMode = false;
    }

    public double normalizeToServoRotations(double radians) {
        double servoAngle = AngleUnit.normalizeRadians(radians * TURRET_GEAR_TO_SERVO_RATIO);

        return (servoAngle - MIN_SERVO_RADIANS) / (MAX_SERVO_RADIANS - MIN_SERVO_RADIANS);
    }
}
