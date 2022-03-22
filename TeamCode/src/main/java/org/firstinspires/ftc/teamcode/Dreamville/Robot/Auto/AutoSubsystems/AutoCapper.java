package org.firstinspires.ftc.teamcode.Dreamville.Robot.Auto.AutoSubsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.LinkedHashMap;
import java.util.Map;

@Config
public class AutoCapper {
    private static final Map<String, Object> telemetry = new LinkedHashMap<>();

    public static double upPos = 1, downPos = 0;

    private static Servo cap;

    private enum capMode {
        UP,
        DOWN
    }

    private capMode capState = capMode.UP;
    private static double lastPos = 0;

    public AutoCapper(HardwareMap hardwareMap) {
        cap = hardwareMap.get(Servo.class, "cap");
    }

    public void update() {
        cap.setPosition(0);
        /*
        switch (capState) {
            case UP:
                if (upPos != lastPos) {
                    cap.setPosition(upPos);
                }
                lastPos = upPos;
                break;
            case DOWN:
                if (downPos != lastPos) {
                    cap.setPosition(downPos);
                }
                lastPos = downPos;
                break;
        }

         */
    }

    public void raise() {
        capState = capMode.UP;
    }

    public void lower() {
        capState = capMode.DOWN;
    }

    public Map<String, Object> getTelemetry() {
        return telemetry;
    }
}