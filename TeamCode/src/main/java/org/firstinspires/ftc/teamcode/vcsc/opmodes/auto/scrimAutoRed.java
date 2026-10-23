package org.firstinspires.ftc.teamcode.vcsc.opmodes.auto;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.vcsc.subsystems.posesystems.PoseConstants;
import org.firstinspires.ftc.teamcode.vcsc.subsystems.posesystems.PoseStorage;

@Autonomous(name = "scrimAutoRed", preselectTeleOp = "scrimTeleOpBlue")
public class scrimAutoRed extends OpMode {
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

    // private RevColorSensorV3 ledStick;
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

        // ledStick = hardwareMap.get(RevColorSensorV3.class, "ledStick");
        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        flywheel = hardwareMap.get(DcMotorEx.class, "scrimFlywheel");
    }
    // MAPPINGS ------------------------

    private Follower follower;

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

    PIDFCoefficients flywheelPIDF = new PIDFCoefficients(p, i, d, f);
    public static double p = 36;
    public static double i = 1;
    public static double d = 28.5;
    public static double f = 20;
    public static double flywheelRPM = 2450;

    private final Pose start = PoseConstants.p.of(58, 8, 90);
    private final Pose path1 = PoseConstants.p.of(49.3661, 15.8204, 180);
    private final Pose point2Start = PoseConstants.p.of(49.3661, 15.8204, 180);
    private final Pose point2 = PoseConstants.p.of(8.5, 7, 180);
    private final Pose point2Control1 = PoseConstants.p.of(45.6209, 6.1779, 0);
    private final Pose point3Start = PoseConstants.p.of(8.5, 7, 180);
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
        //return Paths.line(point4Start, point4).linear(point4Start, point4);
        return Paths.line(point4Start, point4).linear(point4Start, point4);
    }

    @Override
    public void init() {
        hardwareMap();

        follower = Constants.create(hardwareMap);
        // poses
        // RED -
        // BLUE -
        follower.setPose(PoseConstants.p.of(58,8,90));

        pathTimer = new Timer();

        currentState = AllStates.startFlywheel;

        ballStop.setPosition(0.7);

        PoseStorage.autoEnd = PoseConstants.p.of(0,0,0);
    }

    @Override
    public void start() {
        pathTimer.reset();
        frontRamp.setPosition(1);
    }

    @Override
    public void loop() {
        follower.update();

        updatePathState();

        updateTelemetry(telemetry);
        telemetry.update();

        flywheel.setVelocity(rpmToTPS(flywheelRPM));
        flywheelPIDF.p = p;
        flywheelPIDF.i = i;
        flywheelPIDF.d = d;
        flywheelPIDF.f = f;

        telemetry.addData("Pose", follower.pose());
        telemetry.addData("isClose", isClose());
        telemetry.addData("isBusy", follower.isBusy());
    }

    private Pose targetPose = new Pose(0,0,0);

    public void updatePathState() {
        switch (currentState) {
            case startFlywheel:
                ballStop.setPosition(0.7);

                flywheelRPM = 2450;

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

                targetPose = forwardAfterShoot().endPose();

                if (follower.isBusy() || pathTimer.seconds() > 1) {
                    currentState = AllStates.goToIntaking;
                    pathTimer.reset();
                }
                break;
            case goToIntaking:
                intake.setPower(-1);
                inner.setPower(-1);

                follower.follow(goIntakePath());

                targetPose = goIntakePath().endPose();

                if (pathTimer.seconds() > 3.5) {
                    currentState = AllStates.goToHalfOfParking;
                    pathTimer.reset();
                }
                break;
            case goToHalfOfParking:
                follower.follow(halfOfParking());

                targetPose = halfOfParking().endPose();

                if (follower.isBusy() || pathTimer.seconds() > 1) {
                    currentState = AllStates.goToFullParking;
                    pathTimer.reset();
                }
                break;
            case goToFullParking:
                follower.follow(fullParking());

                intake.setPower(1);
                inner.setPower(0);

                targetPose = fullParking().endPose();

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

                break;
        }
    }

    public double rpmToTPS(double rpm) {
        final double TICKS_PER_REVOLUTION = 28.0;
        return (rpm * TICKS_PER_REVOLUTION) / 60;
    }

    public boolean isClose() {
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
    }
}