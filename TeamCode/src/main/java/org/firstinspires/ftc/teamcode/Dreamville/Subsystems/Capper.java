package org.firstinspires.ftc.teamcode.Dreamville.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Capper {
    private boolean g1x;
    private boolean up = false, oldg1x = false;

    public static double bottomPos = 1;
    public static double startPos = 0;

    private Telemetry telemetry;

    private static Servo cap;

    public Capper(HardwareMap hardwareMap) {
        cap = hardwareMap.get(Servo.class, "cap");
    }

    public void cap(boolean g1x, Telemetry telemetry) {
        this.g1x = g1x;

        this.telemetry = telemetry;

        controls();
    }

    public void controls() {
        if (g1x && !oldg1x) {
            up = !up;
        }

        if (up) {
            cap.setPosition(bottomPos);
            telemetry.addData("capperState", "down");
        } else {
            cap.setPosition(startPos);
            telemetry.addData("capperState", "up");
        }

        oldg1x = g1x;
    }
}