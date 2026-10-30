package org.firstinspires.ftc.teamcode.vcsc.opmodes.auto;

import static org.firstinspires.ftc.teamcode.vcsc.subsystems.Shared.SnakeColors;
import static org.firstinspires.ftc.teamcode.vcsc.subsystems.Shared.rpmToTPS;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.vcsc.external.goBildaPrism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.vcsc.subsystems.alliance.currentAlliance;
import org.firstinspires.ftc.teamcode.vcsc.subsystems.posesystems.PoseConstants;
import org.firstinspires.ftc.teamcode.vcsc.subsystems.posesystems.PoseStorage;
import org.firstinspires.ftc.teamcode.vcsc.subsystems.turret.odoAim;

// temp shared auto
public class Auto extends OpMode {
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

    private enum AllStates {
        startFlywheel,
        startShoot,
        endShoot,
        goToForwardAfterShoot,
        goToIntaking,
        goToHalfOfParking,
        goToFullParking,
        end
    }

    private AllStates currentState;

    private Timer pathTimer;
    private Timer matchTimer;

    PIDFCoefficients flywheelPIDF = new PIDFCoefficients(p, i, d, f);
    public static double p = 36;
    public static double i = 1;
    public static double d = 28.5;
    public static double f = 20;
    private static double flywheelRPM = 2390;

    private final Pose start = PoseConstants.p.of(58, 8, 90);
    private final Pose path1 = PoseConstants.p.of(49.3661, 15.8204, 180);
    private final Pose point2Start = PoseConstants.p.of(49.3661, 15.8204, 180);
    private final Pose point2 = PoseConstants.p.of(8.5, 8.5, 180);
    private final Pose point2Control1 = PoseConstants.p.of(45.6209, 8.5, 0);
    private final Pose point3Start = PoseConstants.p.of(8.5, 8.5, 180);
    private final Pose point3 = PoseConstants.p.of(28.1865, 49.9137, 180);
    private final Pose point4Start = PoseConstants.p.of(28.1865, 49.9137, 180);
    private final Pose point4 = PoseConstants.p.of(10.4471, 95.1985, 90);

    public Path forwardAfterShoot() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path goIntakePath() {
        return Paths.curve(point2Start, point2Control1, point2).linear(point2Start, point2);
    }

    public Path halfOfParking() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
    }

    public Path fullParking() {
        return Paths.line(point4Start, point4).linear(point4Start, point4);
    }

    @Override
    public void init() {
        hardwareMap();
        follower = Constants.create(hardwareMap);
        follower.setPose(PoseConstants.p.of(58,8,90));

        OdoAim = new odoAim(hardwareMap, follower, currentAlliance.allianceColor, intake, telemetry);
        OdoAim.manual();

        pathTimer = new Timer();
        matchTimer = new Timer();

        currentState = AllStates.startFlywheel;

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

                flywheelRPM = 2390;

                intake.setPower(-1);
                inner.setPower(-1);

                if (pathTimer.seconds() > 4) {
                    ballStop.setPosition(0.9);
                    currentState = AllStates.startShoot;
                    pathTimer.reset();
                }
                break;
            case startShoot:
                intake.setPower(-1);
                inner.setPower(-1);

                if (pathTimer.seconds() > 3.5) {
                    currentState = AllStates.endShoot;
                    pathTimer.reset();
                }
                break;
            case endShoot:
                ballStop.setPosition(0.7);

                flywheelRPM = 0;

                inner.setPower(0);
                intake.setPower(1);

                if (pathTimer.seconds() > .5) {
                    currentState = AllStates.goToForwardAfterShoot;
                    pathTimer.reset();
                }
                break;
            case goToForwardAfterShoot:
                follower.follow(forwardAfterShoot());

                if (follower.isBusy() || pathTimer.seconds() > 1) {
                    currentState = AllStates.goToIntaking;
                    pathTimer.reset();
                }
                break;
            case goToIntaking:
                intake.setPower(-1);
                inner.setPower(-1);

                follower.follow(goIntakePath());

                if (pathTimer.seconds() > 3.5) {
                    currentState = AllStates.goToHalfOfParking;
                    pathTimer.reset();
                }
                break;
            case goToHalfOfParking:
                follower.follow(halfOfParking());

                if (follower.isBusy() || pathTimer.seconds() > 1) {
                    currentState = AllStates.goToFullParking;
                    pathTimer.reset();
                }
                break;
            case goToFullParking:
                follower.follow(fullParking());

                intake.setPower(1);
                inner.setPower(0);

                if (follower.isBusy() || pathTimer.seconds() > 2.5) {
                    currentState = AllStates.end;
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