package org.firstinspires.ftc.teamcode.vcsc.opmodes.teleOp;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.sparkfun.SparkFunLEDStick;
import com.qualcomm.robotcore.eventloop.opmode.Disabled; // temp
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "scrimTeleRed")
@Disabled // temp for tuning
public class scrimTeleRed extends OpMode {
    // MAPPINGS ------------------------
    private DcMotorEx frontRight;
    private DcMotorEx frontLeft;
    private DcMotorEx backRight;
    private DcMotorEx backLeft;

    private DcMotorEx intake;
    // Inner is the shooter motor
    private DcMotorEx inner;

    private Servo turretRight;
    private Servo turretLeft;
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

        turretRight = hardwareMap.get(Servo.class, "turretRight");
        turretLeft = hardwareMap.get(Servo.class, "turretleft");
        ballStop = hardwareMap.get(Servo.class, "ballStop");
        frontRamp = hardwareMap.get(Servo.class, "frontRamp");

        ledStick = hardwareMap.get(RevColorSensorV3.class, "ledStick");
        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    }
    // MAPPINGS ------------------------

    private Follower follower;

    @Override
    public void init() {
        hardwareMap();

        follower = Constants.create(hardwareMap);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        // Driving
        /*follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x * 0.5,
                true,0
        );*/
    }
}
