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
    private double targetServoHeading;

    // TODO: set the maximum and minimum rotation range of the servos based on how they were configured in the servo programmer
    private final double MIN_SERVO_RADIANS = Math.toRadians(-250); // current range of the servos (NOT THE TURRET) are -175 to 175 degrees
    private final double MAX_SERVO_RADIANS = Math.toRadians(250);

    private final double TURRET_GEAR_TO_SERVO_RATIO = 167.0 / 240.0; // TODO: replace 1/1 with (turret gear teeth) / (servo gear teeth)

    private boolean isManualMode; // TODO: add a manual mode toggle bound to second controller for backup if automatic tracking fails

    // the PedroPathing coordinates of each hive
    final Pose REDTARGETAUDIENCE = new Pose(58, 54);
    final Pose REDTARGETSCORING = new Pose(58,90);
    final Pose BLUETARGETAUDIENCE = new Pose(86,54);
    final Pose BLUETARGETSCORING = new Pose(86,90);

    // odoAim initialization
    public odoAim(HardwareMap hardwareMap, Follower follower, AllianceColor allianceColor) {

        // changes tracked target depending on if we are on blue or red
        this.audienceTargetPose = allianceColor == AllianceColor.RED ? REDTARGETAUDIENCE : BLUETARGETAUDIENCE;
        this.scoringTargetPose = allianceColor == AllianceColor.RED ? REDTARGETSCORING : BLUETARGETSCORING;

        // hardware maps the turret heading servos
        turretHeadingRight = hardwareMap.get(Servo.class, "rightTurret");
        turretHeadingLeft = hardwareMap.get(Servo.class, "leftTurret");
        turretHeadingRight.setDirection(Servo.Direction.REVERSE);
        turretHeadingLeft.setDirection(Servo.Direction.REVERSE);

        // passes the follower into the instance of this class
        this.follower = follower;
    }

    // odoAim looped method
    public void update() {
        // changes the target to either the scoring side or the audience side depending on what side our robot is on
        this.targetPose = follower.pose().y() > 72 ? scoringTargetPose : audienceTargetPose;

        // converts PedroPathing coordinates into robot relative coordinates
        double robotRelativeTargetX = targetPose.x() - follower.pose().x();
        double robotRelativeTargetY = targetPose.y() - follower.pose().y();

        // calculates the angle needed to rotate the turret to those coordinates
        double robotRelativeTargetAngle = Math.atan2(robotRelativeTargetY, robotRelativeTargetX);

        /*
        determines how far the turret needs to rotate to reach the target angle by subtracting how much
        the robot has currently rotated and normalizing it to be between -pi to pi (-180 to 180)
         */
        this.targetTurretHeading = AngleUnit.normalizeRadians(robotRelativeTargetAngle - Math.toRadians(follower.pose().heading()));

        // converts target turret heading to the target servo heading by using the gear ratio of the turret to servo gears
        this.targetServoHeading = this.targetTurretHeading / TURRET_GEAR_TO_SERVO_RATIO;

        // sets the servo angle and prevents the servo from moving past its limits
        if (!isManualMode) {
            if (targetServoHeading <= MAX_SERVO_RADIANS && targetServoHeading >= MIN_SERVO_RADIANS) {
                turretHeadingRight.setPosition(normalizeToServoRotations(targetServoHeading));
                turretHeadingLeft.setPosition(normalizeToServoRotations(targetServoHeading));

            } else if (targetServoHeading > MAX_SERVO_RADIANS) {
                turretHeadingRight.setPosition(normalizeToServoRotations(MAX_SERVO_RADIANS));
                turretHeadingLeft.setPosition(normalizeToServoRotations(MAX_SERVO_RADIANS));

            } else if (targetServoHeading < MIN_SERVO_RADIANS) {
                turretHeadingRight.setPosition(normalizeToServoRotations(MIN_SERVO_RADIANS));
                turretHeadingLeft.setPosition(normalizeToServoRotations(MIN_SERVO_RADIANS));
            }
        }
    }

    // these two methods toggle manual turret aiming on and off (manual mode is not created yet)
    public void auto() {
        this.isManualMode = false;
    }
    public void manual() {
        this.isManualMode = true;
    }

    // converts the degree angle to a scale of 0.0-1.0 to set the servo position
    public double normalizeToServoRotations(double radians) {
        return (radians - MIN_SERVO_RADIANS) / (MAX_SERVO_RADIANS - MIN_SERVO_RADIANS);
    }
}
