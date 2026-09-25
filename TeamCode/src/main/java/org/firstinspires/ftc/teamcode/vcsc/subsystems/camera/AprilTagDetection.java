package org.firstinspires.ftc.teamcode.vcsc.subsystems.camera;

import static org.firstinspires.ftc.teamcode.vcsc.subsystems.camera.AprilTagStorage.limelight3A;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Have to restart every opmode change (run init in init and then loop in loop)
public class AprilTagDetection {
    public AllianceColor allianceColor;

    private int audienceTagNum;
    private int scoringTagNum;

    public void init(HardwareMap hardwareMap, AllianceColor color) {
        limelight3A = null;

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");

        // April Tag Detection is pipeline 1
        limelight3A.pipelineSwitch(1);

        allianceColor = color;

        switch (allianceColor) {
            case RED:
                    scoringTagNum = 32;
                    audienceTagNum = 35;
                break;

            case BLUE:
                    scoringTagNum = 44;
                    audienceTagNum = 39;
                break;
        }
    }

    public void loop() {

    }

    public void endAuto() {

    }

    public void endTeleop() {

    }
}
