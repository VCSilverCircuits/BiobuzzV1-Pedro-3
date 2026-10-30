package org.firstinspires.ftc.teamcode.vcsc.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.vcsc.subsystems.alliance.AllianceColor;
import org.firstinspires.ftc.teamcode.vcsc.subsystems.alliance.currentAlliance;

@Autonomous(preselectTeleOp = "TeleOpBlue")
public class AutoBlue extends Auto {
    public AutoBlue() {
        currentAlliance.allianceColor = AllianceColor.BLUE;
    }
}
