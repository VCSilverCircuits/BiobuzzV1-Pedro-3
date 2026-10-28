package org.firstinspires.ftc.teamcode.vcsc.subsystems.turret;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.vcsc.subsystems.alliance.AllianceColor;

// NOTE: I wrote most of this at 1AM, so the names of some variables may need to be changed to make more sense
@Configurable
public class odoAim {
    private Servo turretHeadingRight;
    private Servo turretHeadingLeft;
    private Follower follower;
    private Limelight3A limelight;

    private Telemetry telemetry;

    private Pose targetPose;
    private Pose audienceTargetPose;
    private Pose scoringTargetPose;

    private double targetTurretHeading = 0;
    private double targetServoHeading = 0;
    private static double MIN_SERVO_RADIANS = Math.toRadians(-177.5); // current range of the servos (NOT THE TURRET) are -177.5 to 177.5 degrees
    private static double MAX_SERVO_RADIANS = Math.toRadians(177.5);

    private final double TURRET_GEAR_TO_SERVO_RATIO = 167.0 / 240.0; // TODO: replace 1/1 with (turret gear teeth) / (servo gear teeth)

    private boolean isManualMode; // TODO: add a manual mode toggle bound to second controller for backup if automatic tracking fails

    // the PedroPathing coordinates of each hive
    final Pose REDTARGETAUDIENCE = new Pose(58, 54); // red aud
    final Pose REDTARGETSCORING = new Pose(58,90); // red sco
    final Pose BLUETARGETAUDIENCE = new Pose(86,54); // blue aud
    final Pose BLUETARGETSCORING = new Pose(86,90); // blue sco

    // odoAim initialization
    public odoAim(HardwareMap hardwareMap, Follower follower, AllianceColor allianceColor, Telemetry telemetry) {
        this.telemetry = telemetry;

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
        this.targetPose = this.follower.pose().y() > 72 ? scoringTargetPose : audienceTargetPose;
    }

    // odoAim looped method
    public void update() {
        this.follower = follower;
        // changes the target to either the scoring side or the audience side depending on what side our robot is on
        this.targetPose = this.follower.pose().y() > 72 ? scoringTargetPose : audienceTargetPose;

        // converts PedroPathing coordinates into robot relative coordinates
        double robotRelativeTargetX = this.targetPose.x() - this.follower.pose().x();
        double robotRelativeTargetY = this.targetPose.y() - this.follower.pose().y();

        // calculates the angle needed to rotate the turret to those coordinates
        double robotRelativeTargetAngle = Math.atan2(robotRelativeTargetY, robotRelativeTargetX);

        /*
        determines how far the turret needs to rotate to reach the target angle by subtracting how much
        the robot has currently rotated and normalizing it to be between -pi to pi (-180 to 180 degrees)
         */
        this.targetTurretHeading = AngleUnit.normalizeRadians(
                robotRelativeTargetAngle - this.follower.pose().heading()
        );

        // converts target turret heading to the target servo heading by using the gear ratio of the turret to servo gears
        this.targetServoHeading = this.targetTurretHeading * TURRET_GEAR_TO_SERVO_RATIO;

        // sets the servo angle and prevents the servo from moving past its limits
        if (!isManualMode) {
            //if (this.targetServoHeading <= MAX_SERVO_RADIANS && this.targetServoHeading >= MIN_SERVO_RADIANS) {
                turretHeadingRight.setPosition(convertToServoRotations(this.targetServoHeading));
                turretHeadingLeft.setPosition(convertToServoRotations(this.targetServoHeading));

            /*} else if (this.targetServoHeading > MAX_SERVO_RADIANS) {
                turretHeadingRight.setPosition(convertToServoRotations(MAX_SERVO_RADIANS));
                turretHeadingLeft.setPosition(convertToServoRotations(MAX_SERVO_RADIANS));

            } else if (this.targetServoHeading < MIN_SERVO_RADIANS) {
                turretHeadingRight.setPosition(convertToServoRotations(MIN_SERVO_RADIANS));
                turretHeadingLeft.setPosition(convertToServoRotations(MIN_SERVO_RADIANS));
            }*/
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
    public double convertToServoRotations(double radians) {
        return (radians - MIN_SERVO_RADIANS) / (MAX_SERVO_RADIANS - MIN_SERVO_RADIANS);
    }

    public void telemetry() {
        telemetry.addData("Robot X", this.follower.pose().x());
        telemetry.addData("Robot Y", this.follower.pose().y());

        telemetry.addData("Target X", this.targetPose.x());
        telemetry.addData("Target Y", this.targetPose.y());

        telemetry.addData("dx", this.targetPose.x() - this.follower.pose().x());
        telemetry.addData("dy", this.targetPose.y() - this.follower.pose().y());

        telemetry.addData("Field Angle (deg)",
                Math.toDegrees(Math.atan2(
                        this.targetPose.y() - this.follower.pose().y(),
                        this.targetPose.x() - this.follower.pose().x()
                )));

        telemetry.addData("Turret Target (deg)",
                Math.toDegrees(this.targetTurretHeading));
    }
}
