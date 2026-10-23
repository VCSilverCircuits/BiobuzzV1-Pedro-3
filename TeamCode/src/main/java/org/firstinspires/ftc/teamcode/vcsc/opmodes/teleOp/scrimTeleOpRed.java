package org.firstinspires.ftc.teamcode.vcsc.opmodes.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vcsc.subsystems.alliance.AllianceColor;
import org.firstinspires.ftc.teamcode.vcsc.subsystems.alliance.currentAlliance;

@TeleOp
public class scrimTeleOpRed extends scrimTeleOp {
    public scrimTeleOpRed() {
        currentAlliance.allianceColor = AllianceColor.RED;
    }
}
