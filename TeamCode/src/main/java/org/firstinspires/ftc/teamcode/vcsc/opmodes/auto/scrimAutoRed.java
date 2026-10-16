package org.firstinspires.ftc.teamcode.vcsc.opmodes.auto;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
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

@Autonomous(name = "scrimAutoRed", preselectTeleOp = "scrimTeleOp")
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

    /*private enum AllStates {
        startFlywheel,
        startShoot,
        endShoot,
        goToForwardAfterShoot,
        goToIntaking,
        goToHalfOfParking,
        goToFullParking,
        end
    }

    private AllStates currentState;*/
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
    public Path forwardAfterShoot = Paths.line(start, path1).linear(start, path1);
    public Path goIntakePath = Paths.curve(point2Start, point2Control1, point2).linear(point2Start, point2);
    public Path halfOfParking = Paths.line(point3Start, point3).linear(point3Start, point3);
    public Path fullParking = Paths.line(point4Start, point4).linear(point4Start, point4);

    public Command startFlywheel() {
        return Command.build()
                .setExecute(() -> startFlywheelVoid())
                .setDone(() -> tpsToRPM(flywheel.getVelocity()) > 2350 && tpsToRPM(flywheel.getVelocity()) < 2550);
    }

    private void startFlywheelVoid() {
        ballStop.setPosition(0.7);

        flywheelRPM = 2450;

        intake.setPower(-1);
        inner.setPower(-1);
    }

    public Command startShoot() {
        return Command.build()
                .setExecute(() -> startShootVoid());
    }

    private void startShootVoid() {
        ballStop.setPosition(0.9);

        intake.setPower(-1);
        inner.setPower(-1);
    }

    public Command endShoot() {
        return Command.build()
                .setExecute(() -> endShootVoid());
    }

    private void endShootVoid() {
        ballStop.setPosition(0.7);

        flywheelRPM = 0;

        inner.setPower(-1);
        intake.setPower(-1);
    }

    public Command endOfAuto() {
        return Command.build()
            .setExecute(() -> endOfAutoVoid());
    }

    private void endOfAutoVoid() {
        flywheelRPM = 0;

        intake.setPower(0);
        inner.setPower(0);
        frontRamp.setPosition(1);
    }

    public Command autoRoutine() {
        return sequential(
                startFlywheel(),
                waitMs(4000),
                startShoot(),
                waitMs(3500),
                endShoot(),
                follow(follower, forwardAfterShoot),
                follow(follower, goIntakePath),
                waitMs(1000),
                follow(follower, halfOfParking),
                follow(follower, fullParking),
                endOfAuto()
        );
    }

    @Override
    public void init() {
        hardwareMap();

        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        PoseStorage.autoEnd = PoseConstants.p.of(0,0,0);
        pathTimer = new Timer();
        //currentState = AllStates.startFlywheel;
        ballStop.setPosition(0.7);

        Scheduler.reset();
        follower.update();
    }

    @Override
    public void start() {
        pathTimer.reset();

        Scheduler.schedule(autoRoutine());
        frontRamp.setPosition(1);
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        // updatePathState();

        flywheel.setVelocity(rpmToTPS(flywheelRPM));
        flywheelPIDF.p = p;
        flywheelPIDF.i = i;
        flywheelPIDF.d = d;
        flywheelPIDF.f = f;

        telemetry.addData("Pose", follower.pose());
        telemetry.addData("Auto Routine", Scheduler.isScheduled(autoRoutine()));
        telemetry.addData("Start Flywheel is Scheduled", Scheduler.isScheduled(startFlywheel()));
        telemetry.addData("Follower Mode", follower.mode());

        if (follower.currentPath() != null) {
            telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
            telemetry.addData("Path number", follower.pathIndex());
        }

        telemetry.update();
    }

    /*public void updatePathState() {
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

                if (pathTimer.seconds() > 1) {
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

                if (pathTimer.seconds() > 1) {
                    currentState = AllStates.goToFullParking;
                    pathTimer.reset();
                }
                break;
            case goToFullParking:
                follower.follow(fullParking());

                intake.setPower(1);
                inner.setPower(0);

                targetPose = fullParking().endPose();

                if (pathTimer.seconds() > 2.5) {
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
    }*/

    public double rpmToTPS(double rpm) {
        final double TICKS_PER_REVOLUTION = 28.0;
        return (rpm * TICKS_PER_REVOLUTION) / 60;
    }

    public double tpsToRPM(double tps) {
        final double TICKS_PER_REVOLUTION = 28.0;
        return (tps * 60) / TICKS_PER_REVOLUTION;
    }

    @Override
    public void stop() {
        PoseStorage.autoEnd = follower.pose();
    }
}
