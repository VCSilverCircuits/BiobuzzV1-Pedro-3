package org.firstinspires.ftc.teamcode.vcsc.opmodes.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vcsc.subsystems.alliance.AllianceColor;
import org.firstinspires.ftc.teamcode.vcsc.subsystems.alliance.currentAlliance;

@TeleOp
public class scrimTeleOpBlue extends scrimTeleOp {
    public scrimTeleOpBlue() {
        currentAlliance.allianceColor = AllianceColor.BLUE;
    }
}
