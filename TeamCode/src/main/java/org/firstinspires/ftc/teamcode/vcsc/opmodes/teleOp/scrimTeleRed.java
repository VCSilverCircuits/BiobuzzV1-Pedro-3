package org.firstinspires.ftc.teamcode.vcsc.opmodes.teleOp;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "scrimTeleRed")
public class scrimTeleRed extends OpMode {
    private Follower follower;

    private enum Actions {
        INTAKING,
        BACKSPIN,
        SHOOTING,
    }

    private Actions currentAction;
    private Actions previousAction;

    private boolean heldShooting;

    @Override
    public void init() {
        hardwareMap();

        follower = Constants.create(hardwareMap);

        currentAction = Actions.INTAKING;
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

        // Actions

        switch (currentAction) {
            case INTAKING:
                ballStop.setPosition(0.7);

                intake.setPower(-1);
                inner.setPower(-1);
            case BACKSPIN:
                ballStop.setPosition(0.7);

                intake.setPower(1);
                inner.setPower(0);
            case SHOOTING:
                ballStop.setPosition(0);

                intake.setPower(0);
                inner.setPower(-1);
        }

        if (gamepad1.right_bumper) {
            if (currentAction == Actions.INTAKING) {
                currentAction = Actions.BACKSPIN;
            } else if (currentAction == Actions.BACKSPIN) {
                currentAction = Actions.INTAKING;
            }
        }

        if (gamepad1.right_trigger > 0.56) {
            heldShooting = true;

            previousAction = currentAction;

            currentAction = Actions.SHOOTING;
        } else {
            if (heldShooting) {
                heldShooting = false;
                currentAction = previousAction;
            }
        }
    }

    // MAPPINGS ------------------------
    private DcMotorEx frontRight;
    private DcMotorEx frontLeft;
    private DcMotorEx backRight;
    private DcMotorEx backLeft;

    private DcMotorEx intake;
    private DcMotorEx inner;
    // turret motor when added

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
    }
    // MAPPINGS ------------------------
}
