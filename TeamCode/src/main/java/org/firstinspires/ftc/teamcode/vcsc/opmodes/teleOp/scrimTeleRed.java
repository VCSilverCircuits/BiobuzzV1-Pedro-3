package org.firstinspires.ftc.teamcode.vcsc.opmodes.teleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;
@TeleOp(name = "scrimTeleRed")
@Configurable
public class scrimTeleRed extends OpMode {
    private Follower follower;

    private enum ActionsB {
        INTAKING,
        BACKSPIN,
        SHOOTING,
    }

    private static ActionsB currentActionB;
    private static ActionsB previousActionB;

    private boolean heldShooting;

    @Override
    public void init() {
        hardwareMap();

        follower = Constants.create(hardwareMap);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelPIDF);
        currentActionB = ActionsB.INTAKING;

        previousActionB = null;
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        // Driving
        // Robot Centric
        follower.manual(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x * 0.5
        );

        follower.update();
        // Driving

        // ActionsB

        switch (currentActionB) {
            case INTAKING:
                ballStop.setPosition(0.75);

                intake.setPower(-1);
                inner.setPower(-1);
                break;
            case BACKSPIN:
                ballStop.setPosition(0.75);

                intake.setPower(1);
                inner.setPower(0);
                break;
            case SHOOTING:
                ballStop.setPosition(0.9);

                intake.setPower(0);
                inner.setPower(-1);
                break;
        }

        if (gamepad1.right_bumper) {
            currentActionB = ActionsB.INTAKING;
        }

        if (gamepad1.left_bumper) {
            currentActionB = ActionsB.BACKSPIN;
        }

        if (gamepad1.right_trigger > 0.56) {
            heldShooting = true;

            if (previousActionB == null) {
                previousActionB = currentActionB;
            }

            currentActionB = ActionsB.SHOOTING;
        } else {
            if (heldShooting) {
                heldShooting = false;
                currentActionB = previousActionB;
                previousActionB = null;
            }
        }
        // Turn on flywheel velocity on start
        flywheel.setVelocity(rpmToTPS(flywheelRPM));
        flywheelPIDF.p = p;
        flywheelPIDF.i = i;
        flywheelPIDF.d = d;
        flywheelPIDF.f = f;

        //Telemetry
        telemetry.addData("Current Action:", currentActionB);
        telemetry.addData("Flywheel Power:", flywheel.getPower());
        telemetry.addData("Flywheel Velocity:", (flywheel.getVelocity()/28)*60);
    }

    // MAPPINGS ------------------------
    private DcMotorEx frontRight;
    private DcMotorEx frontLeft;
    private DcMotorEx backRight;
    private DcMotorEx backLeft;

    private DcMotorEx intake;
    private DcMotorEx inner;
    // turret motor when added
    private DcMotorEx flywheel;

    /*private Servo turretRight;
    private Servo turretLeft;*/
    private Servo ballStop;
    private Servo frontRamp;


    private RevColorSensorV3 ledStick;
    private GoBildaPinpointDriver pinpointDriver;

    // Flywheel PIDF and other setup
    public static double p = 32;
    public static double i = 1;
    public static double d = 28.5;
    public static double f = 16;
    PIDFCoefficients flywheelPIDF = new PIDFCoefficients(p, i, d, f);
    public static double flywheelRPM = 4700;

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
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public double rpmToTPS(double rpm) {
        final double TICKS_PER_REVOLUTION = 28.0;
        return (rpm * TICKS_PER_REVOLUTION) / 60;
    }
    // MAPPINGS ------------------------
}
