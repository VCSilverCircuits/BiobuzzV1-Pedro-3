package org.firstinspires.ftc.teamcode.vcsc.opmodes.teleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "outreachTeleOpCub")
@Disabled
@Configurable
public class outreachTeleOpCub extends OpMode {
    private Follower follower;

    private static TeleActions currentActionB;
    private static TeleActions previousActionB;

    private boolean heldShooting;

    private boolean rampToggle;
    private boolean rampWasLastPressed;
    private boolean eStop;

    // Flywheel PIDF and other setup
    PIDFCoefficients flywheelPIDF = new PIDFCoefficients(p, i, d, f);
    public static double p = 36;
    public static double i = 1;
    public static double d = 28.5;
    public static double f = 20;
    public static double flywheelRPM = 2000;

    @Override
    public void init() {
        hardwareMap();

        follower = Constants.create(hardwareMap);

        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelPIDF);

        currentActionB = TeleActions.INTAKING;
        previousActionB = null;
    }

    @Override
    public void start() {
        frontRamp.setPosition(1);
        rampWasLastPressed = false;
        rampToggle = false;
    }

    @Override
    public void loop() {
        if (!eStop) {
            // Driving
            // Robot Centric
            follower.manual(
                    -gamepad1.left_stick_y / 2,
                    -gamepad1.left_stick_x / 2,
                    -gamepad1.right_stick_x / 0.5
            );

            follower.update();
            // Driving

            // Actions
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

                    intake.setPower(0);
                    inner.setPower(-1);
                    break;
            }
            // Actions

            // Turn on flywheel velocity on start
            flywheel.setVelocity(rpmToTPS(flywheelRPM));
            flywheelPIDF.p = p;
            flywheelPIDF.i = i;
            flywheelPIDF.d = d;
            flywheelPIDF.f = f;

            // Controls
            if (gamepad1.right_bumper) {
                currentActionB = TeleActions.INTAKING;
            }

            if (gamepad1.left_bumper) {
                currentActionB = TeleActions.BACKSPIN;
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
                frontRamp.setPosition(1);
            }
        }

        if (gamepad2.a) {
            eStop = !eStop;
        }
        // Controls

        //Telemetry
        telemetry.addData("Current Action:", currentActionB);
        telemetry.addData("Flywheel Power:", flywheel.getPower());
        telemetry.addData("Flywheel Velocity:", (flywheel.getVelocity()/28)*60);
        telemetry.addData("Ramp Position", frontRamp.getPosition());
    }

    public double rpmToTPS(double rpm) {
        final double TICKS_PER_REVOLUTION = 28.0;
        return (rpm * TICKS_PER_REVOLUTION) / 60;
    }

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

    private RevColorSensorV3 ledStick;
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

        ledStick = hardwareMap.get(RevColorSensorV3.class, "ledStick");
        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        flywheel = hardwareMap.get(DcMotorEx.class, "scrimFlywheel");
    }
    // MAPPINGS ------------------------
}
