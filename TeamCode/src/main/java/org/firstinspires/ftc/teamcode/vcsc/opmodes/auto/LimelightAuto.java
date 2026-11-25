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
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.vcsc.external.goBildaPrism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.vcsc.subsystems.alliance.AllianceColor;
import org.firstinspires.ftc.teamcode.vcsc.subsystems.alliance.currentAlliance;
import org.firstinspires.ftc.teamcode.vcsc.subsystems.posesystems.PoseConstants;
import org.firstinspires.ftc.teamcode.vcsc.subsystems.posesystems.PoseStorage;
import org.firstinspires.ftc.teamcode.vcsc.subsystems.turret.FlywheelConstants;
import org.firstinspires.ftc.teamcode.vcsc.subsystems.turret.odoAim;

@Autonomous
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
    private Servo hood;

    private GoBildaPrismDriver ledStick;
    private GoBildaPinpointDriver pinpointDriver;

    private Limelight3A limelight;

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
        hood = hardwareMap.get(Servo.class, "hoodAngle");

        ledStick = hardwareMap.get(GoBildaPrismDriver.class, "ledStick");
        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        flywheel = hardwareMap.get(DcMotorEx.class, "scrimFlywheel");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }
    // MAPPINGS ------------------------

    private Follower follower;
    private odoAim OdoAim;
    /**
     * Use limelight to detect if the bucket has tipped <p>
     * This is used for checking which bucket to shoot in second time
     */
    private boolean isTipped;

    private boolean alreadyChecked = false;

    private AllAutoStates currentState;

    private Timer pathTimer;
    private Timer matchTimer;

    PIDFCoefficients flywheelPIDF = new PIDFCoefficients(p, i, d, f);
    public static double p = 39.5;
    public static double i = 0;
    public static double d = 34;
    public static double f = 23;
    public static double flywheelRPM = 3000;

    private final Pose start = PoseConstants.p.of(58, 8, 90);
    private final Pose goToIntake = PoseConstants.p.of(-1, 9, 270);
    private final Pose goToIntakeControl1 = PoseConstants.p.of(5.2893, 62.7583, 0);
    private final Pose goToIntakeSegment1Start = PoseConstants.p.of(-1, 9, 90);
    private final Pose goToIntakeSegment1End = PoseConstants.p.of(-1, 9, 270);
    private final Pose goToIntakeSegment2Start = PoseConstants.p.of(-1, 9, 270);
    private final Pose goToIntakeSegment2End = PoseConstants.p.of(-1, 9, 270);
    private final Pose checkLimelight = PoseConstants.p.of(21.5002, 48.9034, 270);
    private final Pose checkLimelightControl1 = PoseConstants.p.of(42.5865, 6.2738, 0);
    private final Pose checkLimelightSegment1Start = PoseConstants.p.of(21.5002, 48.9034, 270);
    private final Pose checkLimelightSegment1End = PoseConstants.p.of(21.5002, 48.9034, 270);
    private final Pose checkLimelightSegment2Start = PoseConstants.p.of(21.5002, 48.9034, 270);
    private final Pose checkLimelightSegment2End = PoseConstants.p.of(21.5002, 48.9034, 270);
    private final Pose shootTwoTipped = PoseConstants.p.of(46.6006, 119.5955, 90);
    private final Pose shootTwoTippedControl1 = PoseConstants.p.of(30.882, 87.8487, 0);
    private final Pose flowerGrab = PoseConstants.p.of(46.728, 129.75, 90);
    private final Pose goOtherFlower1 = PoseConstants.p.of(21.4684, 109.8301, 90);
    private final Pose goOtherFlower = PoseConstants.p.of(18.8736, 48.8586, -180);
    private final Pose goFowardAfterFlower = PoseConstants.p.of(9, 50, -180);
    private final Pose shootTwoNoTipped = PoseConstants.p.of(57.2058, 8.0568, 90);
    private final Pose halfOfParkPathStart = PoseConstants.p.of(8.5, 9, 180);
    private final Pose halfOfParkPathEnd = PoseConstants.p.of(28.1865, 49.9137, 180);
    private final Pose fullOfParkPathStart = PoseConstants.p.of(28.1865, 49.9137, 180);
    private final Pose fullOfParkPathEnd = PoseConstants.p.of(10.4471, 95.1985, 90);

    public Path goOtherFlowerBack() {
        return Paths.line(goFowardAfterFlower, goOtherFlower).constant(goOtherFlower);
    }

    public Path flowerGrabEndOne() {
        return Paths.line(flowerGrab, shootTwoTipped).constant(flowerGrab);
    }

    public Path goOtherFlower1() {
        return Paths.line(flowerGrab, goOtherFlower1).constant(goOtherFlower1);
    }

    public Path goOtherFlower() {
        return Paths.line(goOtherFlower1, goOtherFlower).constant(goOtherFlower);
    }

    public Path goFowardAfterFlower() {
        return Paths.line(goOtherFlower, goFowardAfterFlower).constant(goFowardAfterFlower);
    }

    public Path goToIntake() {
        return Paths
                .curve(start, goToIntakeControl1, goToIntake)
                .heading(Interpolator.piecewise().until(0.5, Interpolator.linear(goToIntakeSegment1Start, goToIntakeSegment1End)).until(1, Interpolator.linear(goToIntakeSegment2Start, goToIntakeSegment2End)));
    }

    public Path checkLimelight() {
        return Paths
                .curve(goToIntake, checkLimelightControl1, checkLimelight)
                .heading(Interpolator.piecewise().until(0.5, Interpolator.linear(checkLimelightSegment1Start, checkLimelightSegment1End))
                        .until(1, Interpolator.linear(checkLimelightSegment2Start, checkLimelightSegment2End)))
                .with(Constants.foresightConfig.brakeAggression.at(0.5));
    }

    public Path shootTwoTipped() {
        return Paths.curve(checkLimelight, shootTwoTippedControl1, shootTwoTipped).constant(shootTwoTipped);
    }

    public Path flowerGrabOne() {
        return Paths.line(shootTwoTipped, flowerGrab).constant(flowerGrab);
    }

    // Not tipped path
    public Path shootTwoNoTipped() {
        return Paths.line(checkLimelight, shootTwoNoTipped).constant(shootTwoNoTipped);
    }

    // Use if not tipped
    // If tipped do not use
    public Path halfOfParking() {
        return Paths.line(shootTwoNoTipped, halfOfParkPathEnd).linear(halfOfParkPathStart, halfOfParkPathEnd);
    }

    public Path fullParkingNoTip() {
        return Paths.line(fullOfParkPathStart, fullOfParkPathEnd).linear(fullOfParkPathStart, fullOfParkPathEnd);
    }

    // If tipped use
    public Path fullParkingTip() {
        return Paths.line(goOtherFlower, fullOfParkPathEnd).linear(fullOfParkPathStart, fullOfParkPathEnd);
    }

    private int aprilTagIds;

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

        isTipped = false;

        if (currentAlliance.allianceColor == AllianceColor.RED) {
            aprilTagIds = 34;
        } else {
            aprilTagIds = 42;
        }

        limelight.pipelineSwitch(1);
        limelight.start();

        PoseStorage.autoEnd = PoseConstants.p.of(58,8,90);

        SnakeColors(ledStick, currentAlliance.allianceColor);
    }

    @Override
    public void start() {
        pathTimer.reset();
        matchTimer.reset();

        hood.setPosition(0);

        frontRamp.setPosition(0.9);

        OdoAim.startOfAutonomous();
    }

    @Override
    public void loop() {
        follower.update();
        OdoAim.update();
        OdoAim.telemetry();

        updatePathState();

        flywheelRPM = FlywheelConstants.flywheelRPM;
        flywheel.setVelocity(rpmToTPS(flywheelRPM));
        p = FlywheelConstants.p;
        i = FlywheelConstants.i;
        d = FlywheelConstants.d;
        f = FlywheelConstants.f;
        flywheelPIDF.p = p;
        flywheelPIDF.i = i;
        flywheelPIDF.d = d;
        flywheelPIDF.f = f;
        hood.setPosition(FlywheelConstants.hoodAnglePos);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelPIDF);

        telemetry.addData("Match Timer", matchTimer.seconds());
        telemetry.addData("isTipped", isTipped);
        telemetry.addData("Pose", follower.pose());
        telemetry.addData("isBusy", follower.isBusy());
        telemetry.addData("RobotHeading", follower.pose().heading());
    }

    public void updatePathState() {
        // State machine
        switch (currentState) {
            case startFlywheel:
                ballStop.setPosition(0.7);

                if (pathTimer.seconds() > 2) {
                    intake.setPower(-1);
                    inner.setPower(-1);
                }

                if (pathTimer.seconds() > 2.5) {
                    ballStop.setPosition(0.9);
                    currentState = AllAutoStates.startShootOne;
                    pathTimer.reset();
                }
                break;
            case startShootOne:
                intake.setPower(-1);
                inner.setPower(-1);

                if (pathTimer.seconds() > 2) {
                    currentState = AllAutoStates.endShootOne;
                    pathTimer.reset();
                }
                break;
            case endShootOne:
                ballStop.setPosition(0.7);

                inner.setPower(0);
                intake.setPower(1);

                if (pathTimer.seconds() > .2) {
                    currentState = AllAutoStates.goToIntaking;
                    pathTimer.reset();
                }
                break;
            case goToIntaking:
                intake.setPower(-1);
                inner.setPower(-1);

                follower.follow(goToIntake());

                if (pathTimer.seconds() > 3) {
                    currentState = AllAutoStates.limelightPath;
                    pathTimer.reset();
                }
                break;
            case limelightPath:
                follower.follow(checkLimelight());

                if (!follower.isBusy() || pathTimer.seconds() > 1.5) {
                    currentState = AllAutoStates.checkLimelight;
                    alreadyChecked = false;
                    pathTimer.reset();
                }
                break;
            case checkLimelight:
                if (!alreadyChecked) {
                    LLResult result = limelight.getLatestResult();
                    isTipped = true;

                    for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                        int seenId = fiducial.getFiducialId();
                        telemetry.addData("Seen Id", seenId);

                        if (seenId == aprilTagIds) {
                            isTipped = false;
                            ballStop.setPosition(0.7);
                        }
                    }
                    ballStop.setPosition(0.7);
                    inner.setPower(0);
                    intake.setPower(0);
                    alreadyChecked = true;
                }

                if (pathTimer.seconds() > .7) {
                    if (isTipped) {
                        currentState = AllAutoStates.goToShootPosTwoTip;
                        FlywheelConstants.flywheelRPM = 2800;
                        FlywheelConstants.hoodAnglePos = 0.035;
                    } else {
                        currentState = AllAutoStates.goToShootPosTwoNoTip;
                    }
                    pathTimer.reset();
                }
                break;
            case goToShootPosTwoTip:
                inner.setPower(0);
                intake.setPower(0);

                follower.follow(shootTwoTipped());

                if (!follower.isBusy() || pathTimer.seconds() > 1) {
                    currentState = AllAutoStates.startShootTwoTip;
                    pathTimer.reset();
                }
                break;
            case goToShootPosTwoNoTip:
                inner.setPower(0);
                intake.setPower(0);

                ballStop.setPosition(0.7);
                follower.follow(shootTwoNoTipped());
                ballStop.setPosition(0.7);

                if (!follower.isBusy() || pathTimer.seconds() > 4) {
                    currentState = AllAutoStates.startShootTwoNoTip;
                    ballStop.setPosition(0.7);
                    pathTimer.reset();
                }
                break;
            case startShootTwoTip:
                if (pathTimer.seconds() > 2.5) {
                    inner.setPower(-1);
                    intake.setPower(-1);

                    ballStop.setPosition(0.9);
                }
                if (pathTimer.seconds() > 3) {
                    currentState = AllAutoStates.endShootTwoTip;
                    pathTimer.reset();
                }
                break;
            case endShootTwoTip:
                ballStop.setPosition(0.7);

                inner.setPower(0);
                intake.setPower(0);

                if (pathTimer.seconds() > .5) {
                    currentState = AllAutoStates.getFromFlower;
                    pathTimer.reset();
                }
                break;
            case startShootTwoNoTip:
                if (pathTimer.seconds() > 2) {
                    inner.setPower(-1);
                    intake.setPower(-1);
                }

                ballStop.setPosition(0.9);

                if (pathTimer.seconds() > 4) {
                    currentState = AllAutoStates.endShootTwoNoTip;
                    pathTimer.reset();
                }
                break;
            case endShootTwoNoTip:
                ballStop.setPosition(0.7);

                inner.setPower(0);
                intake.setPower(0);

                if (pathTimer.seconds() > .5) {
                    currentState = AllAutoStates.goToHalfOfParking;
                    pathTimer.reset();
                }
                break;
            case getFromFlower:
                intake.setPower(-1);
                inner.setPower(-1);

                frontRamp.setPosition(0.428);

                if (pathTimer.seconds() > 1) {
                    frontRamp.setPosition(0.438);
                    follower.follow(flowerGrabOne());
                }

                if (pathTimer.seconds() > 3) {
                    frontRamp.setPosition(0.428);
                    follower.follow(flowerGrabEndOne());
                }

                if (pathTimer.seconds() > 3.5) {
                    currentState = AllAutoStates.shootThreeStartTip;
                    pathTimer.reset();
                }
                break;
            case shootThreeStartTip:
                ballStop.setPosition(0.9);

                if (pathTimer.seconds() > 1.5) {
                    currentState = AllAutoStates.shootThreeEndTip;
                    pathTimer.reset();
                }
                break;
            case shootThreeEndTip:
                ballStop.setPosition(0.7);

                if (pathTimer.seconds() > 0.5) {
                    currentState = AllAutoStates.goToFlowerTwoPath1;
                    pathTimer.reset();
                }
                break;
            case goToFlowerTwoPath1:
                follower.follow(goOtherFlower1());

                if (!follower.isBusy() || pathTimer.seconds() > 1) {
                    currentState = AllAutoStates.goToFlowerTwo;
                    pathTimer.reset();
                }
                break;
            case goToFlowerTwo:
                follower.follow(goOtherFlower());

                if (!follower.isBusy() || pathTimer.seconds() > 1) {
                    currentState = AllAutoStates.grabFlowerTwo;
                    pathTimer.reset();
                }
                break;
            case grabFlowerTwo:
                if (pathTimer.seconds() < 1.5) {
                    follower.follow(goFowardAfterFlower());
                }

                if (pathTimer.seconds() > 1.75) {
                    frontRamp.setPosition(0.438);
                    follower.follow(flowerGrabOne());
                }

                if (pathTimer.seconds() > 2.2) {
                    frontRamp.setPosition(0.428);
                    follower.follow(goOtherFlowerBack());
                }

                if (pathTimer.seconds() > 4) {
                    follower.stop();
                    currentState = AllAutoStates.afterFlowerTwo;
                    pathTimer.reset();
                }
                break;
            case afterFlowerTwo:
                if (pathTimer.seconds() > 1) {
                    frontRamp.setPosition(0.438);
                }

                if (pathTimer.seconds() > 3) {
                    frontRamp.setPosition(0.428);
                    follower.follow(goOtherFlowerBack());
                }

                if (!follower.isBusy() || pathTimer.seconds() > 3.1) {
                    currentState = AllAutoStates.goToFullParkingTip;
                    pathTimer.reset();
                }
                break;
            case goToHalfOfParking:
                follower.follow(halfOfParking());

                FlywheelConstants.flywheelRPM = 3000;
                FlywheelConstants.hoodAnglePos = 0;

                if (!follower.isBusy() || pathTimer.seconds() > 5) {
                    currentState = AllAutoStates.goToFullParkingNoTip;
                    pathTimer.reset();
                }
                break;
            case goToFullParkingTip:
                frontRamp.setPosition(0.9);
                if (pathTimer.seconds() < 1) {
                    follower.follow(fullParkingTip());
                }

                FlywheelConstants.flywheelRPM = 3000;
                FlywheelConstants.hoodAnglePos = 0;

                if (pathTimer.seconds() > 2) {
                    ballStop.setPosition(0.9);
                    intake.setPower(-1);
                    inner.setPower(-1);
                }

                if (pathTimer.seconds() > 3.5) {
                    currentState = AllAutoStates.end;
                    pathTimer.reset();
                }
                break;
            case goToFullParkingNoTip:
                follower.follow(fullParkingNoTip());

                intake.setPower(-1);
                inner.setPower(-1);
                ballStop.setPosition(0.9);

                if (pathTimer.seconds() > 5.5) {
                    currentState = AllAutoStates.end;
                    pathTimer.reset();
                }
                break;
            case end:
                flywheel.setPower(0);

                intake.setPower(0);
                inner.setPower(0);

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