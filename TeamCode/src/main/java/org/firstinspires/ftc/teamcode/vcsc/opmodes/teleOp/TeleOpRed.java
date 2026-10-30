package org.firstinspires.ftc.teamcode.vcsc.opmodes.teleOp;

import org.firstinspires.ftc.teamcode.vcsc.subsystems.alliance.AllianceColor;
import org.firstinspires.ftc.teamcode.vcsc.subsystems.alliance.currentAlliance;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOpRed extends TeleOp {
    public TeleOpRed() {
        currentAlliance.allianceColor = AllianceColor.RED;
    }
}
