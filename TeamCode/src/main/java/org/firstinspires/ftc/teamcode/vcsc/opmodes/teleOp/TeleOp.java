package org.firstinspires.ftc.teamcode.vcsc.opmodes.teleOp;

import static org.firstinspires.ftc.teamcode.vcsc.subsystems.Shared.MirrorPose;
import static org.firstinspires.ftc.teamcode.vcsc.subsystems.Shared.SnakeColors;
import static org.firstinspires.ftc.teamcode.vcsc.subsystems.Shared.rpmToTPS;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.vcsc.external.goBildaPrism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.vcsc.subsystems.alliance.AllianceColor;
import org.firstinspires.ftc.teamcode.vcsc.subsystems.alliance.currentAlliance;
import org.firstinspires.ftc.teamcode.vcsc.subsystems.posesystems.PoseConstants;
import org.firstinspires.ftc.teamcode.vcsc.subsystems.posesystems.PoseStorage;
import org.firstinspires.ftc.teamcode.vcsc.subsystems.turret.odoAim;

@Configurable
public class TeleOp extends OpMode {
    private Follower follower;
    private odoAim OdoAim;

    private static TeleActions currentActionB;
    private static TeleActions previousActionB;

    private boolean heldShooting;

    private boolean rampToggle;
    private boolean rampWasLastPressed;

    private boolean aimWasLastPressed;
    private boolean odoAimToggle;

    private boolean recalLastPressed;

    private boolean backspinLastPressed;

    Timer waitSecondBeforeStart;

    // Temp servo poses
    private static double servoPos = 0.5;
    public static double hoodAngleTemp = 0.44;

    // Flywheel PIDF and other setup
    PIDFCoefficients flywheelPIDF = new PIDFCoefficients(p, i, d, f);
    public static double p = 36;
    public static double i = 1;
    public static double d = 28.5;
    public static double f = 30;
    private static double flywheelRPM = 2390;

    @Override
    public void init() {
        hardwareMap();

        follower = Constants.create(hardwareMap);
        OdoAim = new odoAim(hardwareMap, follower, currentAlliance.allianceColor, intake, telemetry);
        OdoAim.auto();

        if (PoseStorage.autoEnd != null) {
            follower.setPose(PoseStorage.autoEnd);
        } else {
            if (currentAlliance.allianceColor == AllianceColor.RED) {
                follower.setPose(PoseConstants.p.of(58, 8, 90));
            } else {
                Pose pose = new Pose(58, 8, 90);
                Pose mirror = MirrorPose(pose);
                follower.setPose(mirror);
            }
        }

        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelPIDF);

        currentActionB = TeleActions.IDLE;
        previousActionB = null;

        waitSecondBeforeStart = new Timer();

        SnakeColors(ledStick, currentAlliance.allianceColor);
    }

    @Override
    public void start() {
        frontRamp.setPosition(1);
        hoodAngle.setPosition(0.38);
        rampWasLastPressed = false;
        rampToggle = false;
        odoAimToggle = false;
        aimWasLastPressed = false;
        recalLastPressed = false;
        waitSecondBeforeStart.reset();
    }

    @Override
    public void loop() {
        // Driving ---------
        // Robot Centric
        follower.manual(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x * 0.5
        );

        follower.update();
        OdoAim.update();
        OdoAim.telemetry();
        //double pos = 90.0 / 180.0;
        //turretRight.setPosition(servoPos);
        //turretLeft.setPosition(servoPos);
        // Driving ---------

        // Actions ---------
        switch (currentActionB) {
            case INTAKING:
                ballStop.setPosition(0.7);

                intake.setPower(-1);
                inner.setPower(-1);
                break;
            case BACKSPIN:
                ballStop.setPosition(0.7);

                intake.setPower(1);
                inner.setPower(-1);
                break;
            case SHOOTING:
                ballStop.setPosition(0.9);

                inner.setPower(-1);
                intake.setPower(0);
                break;
            case IDLE:
                ballStop.setPosition(0.7);

                intake.setPower(0);
                inner.setPower(0);
                break;
        }
        // Actions ---------

        // Flywheel ---------
        // Turn on flywheel velocity on start
        flywheel.setVelocity(rpmToTPS(flywheelRPM));
        flywheelPIDF.p = p;
        flywheelPIDF.i = i;
        flywheelPIDF.d = d;
        flywheelPIDF.f = f;
        // Flywheel ---------

        // Controls ---------
        if (gamepad1.left_bumper) {
            currentActionB = TeleActions.BACKSPIN;
        } else if (waitSecondBeforeStart.seconds() > 2) {
            currentActionB = TeleActions.INTAKING;
        }

        if (gamepad1.right_trigger > 0.56) {
            heldShooting = true;

            if (previousActionB == null) {
                previousActionB = currentActionB;
            }

            currentActionB = TeleActions.SHOOTING;
        } else {
            if (heldShooting) {
                heldShooting = false;
                currentActionB = previousActionB;
                previousActionB = null;
            }
        }

        if (gamepad1.y && !rampWasLastPressed) {
            rampToggle = !rampToggle;
        }
        rampWasLastPressed = gamepad1.y;
        if (rampToggle) {
            frontRamp.setPosition(0.4);
        } else {
            frontRamp.setPosition(0.9);
        }

        if (gamepad1.dpad_up && !aimWasLastPressed) {
            odoAimToggle = !odoAimToggle;
        }
        aimWasLastPressed = gamepad1.dpad_up;
        if (odoAimToggle) {
            OdoAim.manual();
        } else {
            OdoAim.auto();
        }

        if (gamepad1.dpad_down && !recalLastPressed) {
            follower.setPose(PoseConstants.p.of(80, 58, 270));
        }
        recalLastPressed = gamepad1.dpad_down;
        // Controls ---------

        // Hood angles ---------
        // hoodAngling.setFlywheelSpeedToHoodAngle(hoodAngle.getPosition(), flywheelPIDF);

        hoodAngle.setPosition(hoodAngleTemp);
        // Hood angles ---------

        // Telemetry ---------
        telemetry.addData("Current Action:", currentActionB);
        telemetry.addData("Flywheel Power:", flywheel.getPower());
        telemetry.addData("Flywheel Velocity:", (flywheel.getVelocity()/28)*60);
        telemetry.addData("Ramp Position:", frontRamp.getPosition());
        telemetry.addData("Hood Angle:", hoodAngle.getPosition());
        telemetry.addData("Alliance Color:", currentAlliance.allianceColor);
        telemetry.addData("Robot Heading:", follower.pose().heading());

        PanelsTelemetry.INSTANCE.getTelemetry();
        PanelsTelemetry.INSTANCE.getFtcTelemetry();
        // Telemetry ---------
    }

    @Override
    public void stop() {
        SnakeColors(ledStick);
        PoseStorage.autoEnd = null;
    }

    // MAPPINGS ------------------------
    private DcMotorEx frontRight;
    private DcMotorEx frontLeft;
    private DcMotorEx backRight;
    private DcMotorEx backLeft;

    private DcMotorEx intake;
    private DcMotorEx inner;
    private DcMotorEx flywheel;

    //private Servo turretRight;
    //private Servo turretLeft;
    private Servo ballStop;
    private Servo frontRamp;
    private Servo hoodAngle;

    private GoBildaPrismDriver ledStick;
    private GoBildaPinpointDriver pinpointDriver;

    public void hardwareMap() {
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        inner = hardwareMap.get(DcMotorEx.class, "inner");

        //turretRight = hardwareMap.get(Servo.class, "rightTurret");
        //turretLeft = hardwareMap.get(Servo.class, "leftTurret");
        ballStop = hardwareMap.get(Servo.class, "ballStop");
        frontRamp = hardwareMap.get(Servo.class, "frontRamp");
        hoodAngle = hardwareMap.get(Servo.class, "hoodAngle");

        ledStick = hardwareMap.get(GoBildaPrismDriver.class, "ledStick");
        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        flywheel = hardwareMap.get(DcMotorEx.class, "scrimFlywheel");
    }
    // MAPPINGS ------------------------
}
