package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(1.536878000094196);
        c.yPodOffset.set(0.18934968888290285);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED); // Has to be reversed or else it tracks backwards
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.28099201225483506);
                Controller secondaryTranslationalForward = Controller.proportional(0.10381904028331276);
                Controller primaryTranslationalLateral = Controller.proportional(0.4151404902911213);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1533833184043705);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.010653258261825234));
                c.brake.set(Controller.proportionalFeedforward(0.009055269522551448));

                c.headingFeedback.set(Controller.proportional(4.032145693440726));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06840199369723082, 0.004301115365649158));

                c.linearBrakeCoefficients.set(Matrix.diag(0.07765201518127758, 0.04743199457969815));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.002416988430378128, 0.00242279315178006));

                c.maxAchievableForwardVelocity.set(84.97227405321348);
                c.maxAchievableStrafeVelocity.set(68.53446882732202);
                c.naturalForwardDeceleration.set(26.449982934667975);
                c.naturalStrafeDeceleration.set(61.72219996365364);

                c.timeoutConstraint.set(500.0);
                c.headingConstraint.set(Math.toRadians(0.4));
            }
    );
}