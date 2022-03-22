package org.firstinspires.ftc.teamcode.Dreamville.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Capper {
    private boolean g1x;
    private boolean oldg1x = false;

    public static double bottomPos = 0.65;
    public static double startPos = 0.25;
    public static double middlePos = 0.405;

    private Telemetry telemetry;

    private static Servo cap;

    enum capState {
        INIT,
        UP,
        MIDDLE,
        BOTTOM
    }

    private capState capMode = capState.INIT;

    public Capper(HardwareMap hardwareMap) {
        cap = hardwareMap.get(Servo.class, "cap");
        cap.setPosition(0);
    }

    public void cap(boolean g1x, Telemetry telemetry) {
        this.g1x = g1x;

        this.telemetry = telemetry;

        controls();
    }

    public void controls() {
        switch (capMode) {
            case INIT:
                if (g1x) {
                    cap.setPosition(startPos);
                    capMode = capState.UP;
                }
                break;
            case UP:
                if (g1x && !oldg1x) {
                    cap.setPosition(bottomPos);
                    capMode = capState.BOTTOM;
                }
                oldg1x = g1x;
                break;
            case MIDDLE:
                if (g1x && !oldg1x) {
                    cap.setPosition(startPos);
                    capMode = capState.UP;
                }
                oldg1x = g1x;
                break;
            case BOTTOM:
                if (g1x && !oldg1x) {
                    cap.setPosition(middlePos);
                    capMode = capState.MIDDLE;
                }
                oldg1x = g1x;
                break;
        }
    }
}