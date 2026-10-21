package org.firstinspires.ftc.teamcode.vcsc.subsystems.turret;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class hoodAngling {
    public static void setFlywheelSpeedToHoodAngle(double hoodAngle, PIDFCoefficients flywheelPIDF, DcMotorEx flywheel) {
        // Switch Cases don't work with doubles :(
        // Worth improving the way it
        if (hoodAngle == 0) {
            // 0 rpm
        } else if (hoodAngle == 0.1 || hoodAngle > 0.1 && hoodAngle < 0.2) {
        } else if (hoodAngle == 0.2 || hoodAngle > 0.2 && hoodAngle < 0.3) {
        } else if (hoodAngle == 0.3 || hoodAngle > 0.3 && hoodAngle < 0.4) {
        } else if (hoodAngle == 0.4 || hoodAngle > 0.4 && hoodAngle < 0.5) {
        } else if (hoodAngle == 0.5 || hoodAngle > 0.5 && hoodAngle < 0.6) {
        } else if (hoodAngle == 0.6 || hoodAngle > 0.6 && hoodAngle < 0.7) {
        } else if (hoodAngle == 0.7 || hoodAngle > 0.7 && hoodAngle < 0.8) {
        } else if (hoodAngle == 0.8 || hoodAngle > 0.8 && hoodAngle < 0.9) {
        } else if (hoodAngle == 0.9 || hoodAngle > 0.9 && hoodAngle < 1) {
        } else if (hoodAngle == 1) {
        }
    }

    private void setFlywheelPIDFs(double p, double i, double d, double f, PIDFCoefficients flywheelPIDF) {
        flywheelPIDF.p = p;
        flywheelPIDF.i = i;
        flywheelPIDF.d = d;
        flywheelPIDF.f = f;
    }

    private void setFlywheelRPM(double rpm, DcMotorEx flywheel) {
        flywheel.setVelocity(rpmToTPS(rpm));
    }

    private double rpmToTPS(double rpm) {
        final double TICKS_PER_REVOLUTION = 28.0;
        return (rpm * TICKS_PER_REVOLUTION) / 60;
    }
}
