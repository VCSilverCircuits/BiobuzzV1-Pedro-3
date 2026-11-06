package org.firstinspires.ftc.teamcode.vcsc.opmodes.auto;

import static org.firstinspires.ftc.teamcode.vcsc.subsystems.Shared.SnakeColors;
import static org.firstinspires.ftc.teamcode.vcsc.subsystems.Shared.rpmToTPS;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.utils.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.vcsc.external.goBildaPrism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.vcsc.subsystems.alliance.currentAlliance;
import org.firstinspires.ftc.teamcode.vcsc.subsystems.posesystems.PoseConstants;
import org.firstinspires.ftc.teamcode.vcsc.subsystems.posesystems.PoseStorage;
import org.firstinspires.ftc.teamcode.vcsc.subsystems.turret.odoAim;

// temp shared auto
public class LimelightAuto extends OpMode {
    // MAPPINGS ------------------------
    private DcMotorEx frontRight;
    private DcMotorEx frontLeft;
    private DcMotorEx backRight;
    private DcMotorEx backLeft;

    private DcMotorEx intake;
    private DcMotorEx inner;
    private DcMotorEx flywheel;

    /*private Servo turretRight;
    private Servo turretLeft;*/
    private Servo ballStop;
    private Servo frontRamp;

    private GoBildaPrismDriver ledStick;
    private GoBildaPinpointDriver pinpointDriver;

    public void hardwareMap() {
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        inner = hardwareMap.get(DcMotorEx.class, "inner");

        /*turretRight = hardwareMap.get(Servo.class, "turretRight");
        turretLeft = hardwareMap.get(Servo.class, "turretleft");*/
        ballStop = hardwareMap.get(Servo.class, "ballStop");
        frontRamp = hardwareMap.get(Servo.class, "frontRamp");

        ledStick = hardwareMap.get(GoBildaPrismDriver.class, "ledStick");
        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        flywheel = hardwareMap.get(DcMotorEx.class, "scrimFlywheel");
    }
    // MAPPINGS ------------------------

    private Follower follower;
    private odoAim OdoAim;
    /**
     * Use limelight to detect if the bucket has tipped <p>
     * This is used for checking which bucket to shoot in second time
     */
    private boolean isTipped;

    private AllAutoStates currentState;

    private Timer pathTimer;
    private Timer matchTimer;

    PIDFCoefficients flywheelPIDF = new PIDFCoefficients(p, i, d, f);
    public static double p = 36;
    public static double i = 1;
    public static double d = 28.5;
    public static double f = 30;
    private static double flywheelRPM = 2440;

    private final Pose start = PoseConstants.p.of(58, 8, 90);
    private final Pose goToIntake = PoseConstants.p.of(8.5, 9, 270);
    private final Pose goToIntakeControl1 = PoseConstants.p.of(5.2893, 62.7583, 0);
    private final Pose goToIntakeSegment1Start = PoseConstants.p.of(8.5, 9, 90);
    private final Pose goToIntakeSegment1End = PoseConstants.p.of(8.5, 9, 270);
    private final Pose goToIntakeSegment2Start = PoseConstants.p.of(8.5, 9, 270);
    private final Pose goToIntakeSegment2End = PoseConstants.p.of(8.5, 9, 270);
    private final Pose shootTwoTippedStart = PoseConstants.p.of(8.5, 9, 270);
    private final Pose shootTwoTipped = PoseConstants.p.of(58, 133, 270);
    private final Pose shootTwoTippedControl1 = PoseConstants.p.of(35.3182, 52.6558, 0);
    private final Pose shootTwoTippedControl2 = PoseConstants.p.of(15.8399, 139.0456, 0);
    private final Pose shootTwoTippedControl3 = PoseConstants.p.of(55.388, 99.7961, 0);
    private final Pose halfOfParkPathStart = PoseConstants.p.of(8.5, 9, 180);
    private final Pose halfOfParkPathEnd = PoseConstants.p.of(28.1865, 49.9137, 180);
    private final Pose fullOfParkPathStart = PoseConstants.p.of(28.1865, 49.9137, 180);
    private final Pose fullOfParkPathEnd = PoseConstants.p.of(10.4471, 95.1985, 90);

    public Path goIntakePath() {
        return Paths.curve(start, goToIntakeControl1, goToIntake)
                .heading(Interpolator.piecewise()
                        .until(0.5, Interpolator.linear(goToIntakeSegment1Start, goToIntakeSegment1End))
                        .until(1, Interpolator.linear(goToIntakeSegment2Start, goToIntakeSegment2End)));
    }

    // not tipped path
    public Path shootTwoNoTipped() {
        return Paths.curve(goToIntake, goToIntakeControl1, start)
                .heading(Interpolator.piecewise()
                        .until(0.5, Interpolator.linear(goToIntakeSegment1End, goToIntakeSegment1Start))
                        .until(1, Interpolator.linear(goToIntakeSegment2End, goToIntakeSegment2Start)));
    }

    // Tipped path
    public Path shootTwoTipped() {
        return Paths
                .curve(shootTwoTippedStart, shootTwoTippedControl1, shootTwoTippedControl2, shootTwoTippedControl3, shootTwoTipped)
                .linear(shootTwoTippedStart, shootTwoTipped);
    }

    // Use if not tipped
    // If tipped do not use
    public Path halfOfParking() {
        return Paths.line(halfOfParkPathStart, halfOfParkPathEnd).linear(halfOfParkPathStart, halfOfParkPathEnd);
    }

    public Path fullParking() {
        return Paths.line(fullOfParkPathStart, fullOfParkPathEnd).linear(fullOfParkPathStart, fullOfParkPathEnd);
    }



    @Override
    public void init() {
        hardwareMap();
        follower = Constants.create(hardwareMap);
        follower.setPose(PoseConstants.p.of(58,8,90));

        OdoAim = new odoAim(hardwareMap, follower, currentAlliance.allianceColor, intake, telemetry);
        OdoAim.auto();

        pathTimer = new Timer();
        matchTimer = new Timer();

        currentState = AllAutoStates.startFlywheel;

        ballStop.setPosition(0.7);

        PoseStorage.autoEnd = PoseConstants.p.of(58,8,90);

        SnakeColors(ledStick, currentAlliance.allianceColor);
    }

    @Override
    public void start() {
        pathTimer.reset();
        matchTimer.reset();

        frontRamp.setPosition(0.9);
    }

    @Override
    public void loop() {
        follower.update();
        OdoAim.update();
        OdoAim.telemetry();

        updatePathState();

        flywheel.setVelocity(rpmToTPS(flywheelRPM));
        flywheelPIDF.p = p;
        flywheelPIDF.i = i;
        flywheelPIDF.d = d;
        flywheelPIDF.f = f;

        telemetry.addData("Pose", follower.pose());
        telemetry.addData("isBusy", follower.isBusy());
        telemetry.addData("RobotHeading", follower.pose().heading());
    }

    public void updatePathState() {
        // IVY and state machine
        switch (currentState) {
            case startFlywheel:
                ballStop.setPosition(0.7);

                flywheelRPM = 2440;

                intake.setPower(-1);
                if (pathTimer.seconds() > 2) {
                    inner.setPower(-1);
                }

                if (pathTimer.seconds() > 4) {
                    ballStop.setPosition(0.9);
                    currentState = AllAutoStates.startShootOne;
                    pathTimer.reset();
                }
                break;
            case startShootOne:
                intake.setPower(-1);
                inner.setPower(-1);

                if (pathTimer.seconds() > 3.5) {
                    currentState = AllAutoStates.endShootOne;
                    pathTimer.reset();
                }
                break;
            case endShootOne:
                ballStop.setPosition(0.7);

                inner.setPower(0);
                intake.setPower(1);

                if (pathTimer.seconds() > .5) {
                    currentState = AllAutoStates.goToIntaking;
                    pathTimer.reset();
                }
                break;
            /*case goToForwardAfterShoot:
                follower.follow(forwardAfterShoot());

                if (follower.isBusy() || pathTimer.seconds() > 1) {
                    currentState = AllAutoStates.goToIntaking;
                    pathTimer.reset();
                }
                break;*/
            case goToIntaking:
                intake.setPower(-1);
                inner.setPower(-1);

                follower.follow(goIntakePath());

                if (pathTimer.seconds() > 3.5) {
                    currentState = AllAutoStates.goToShootPosTwo;
                    pathTimer.reset();
                }
                break;
            case goToShootPosTwo:
                if (isTipped) {
                    follower.follow(shootTwoTipped());
                } else {
                    follower.follow(shootTwoNoTipped());
                }
                if (follower.isBusy() || pathTimer.seconds() > 4) {
                    currentState = AllAutoStates.goToFullParking;
                    pathTimer.reset();
                }
                break;
            case startShootTwo:
                intake.setPower(-1);
                inner.setPower(-1);

                if (pathTimer.seconds() > 3.5) {
                    currentState = AllAutoStates.endShootTwo;
                    pathTimer.reset();
                }
                break;
            case endShootTwo:
                ballStop.setPosition(0.7);

                flywheelRPM = 0;

                inner.setPower(0);
                intake.setPower(1);

                if (pathTimer.seconds() > .5) {
                    if (isTipped) {
                        currentState = AllAutoStates.goToFullParking;
                    } else {
                        currentState = AllAutoStates.goToHalfOfParking;
                    }
                    pathTimer.reset();
                }
                break;
            case goToHalfOfParking:
                follower.follow(halfOfParking());

                if (follower.isBusy() || pathTimer.seconds() > 1) {
                    currentState = AllAutoStates.goToFullParking;
                    pathTimer.reset();
                }
                break;
            case goToFullParking:
                follower.follow(fullParking());

                intake.setPower(1);
                inner.setPower(0);

                if (follower.isBusy() || pathTimer.seconds() > 2.5) {
                    currentState = AllAutoStates.end;
                    pathTimer.reset();
                }
                break;
            case end:
                flywheelRPM = 0;

                intake.setPower(0);
                inner.setPower(0);
                frontRamp.setPosition(1);

                PoseStorage.autoEnd = follower.pose();

                break;
        }
    }

    /*public boolean isClose() {
        Pose currentPose = follower.pose();
        double distance = Math.hypot(targetPose.x() - currentPose.y(), targetPose.y() - currentPose.y());

        if (distance < 3.0) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void stop() {
        PoseStorage.autoEnd = follower.pose();
    }*/
}