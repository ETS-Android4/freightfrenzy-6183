package org.firstinspires.ftc.teamcode.Dreamville.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

@Config
public class Rumbler {
    private static Gamepad g1;

    public Rumbler(Gamepad gamepad) {
        this.g1 = gamepad;
    }

    public void rumble(double power1, double power2, int time) {
        if (!g1.isRumbling()) {
            g1.rumble(power1, power2, time);
        }
    }

    public void rumble(int time) {
        if (!g1.isRumbling()) {
            g1.rumble(time);
        }
    }

    public void rumbleBlips(int count) {
        if (!g1.isRumbling()) {
            g1.rumbleBlips(count);
        }
    }

    public void stopRumble() {
        g1.stopRumble();
    }

    public boolean isRumbling() {
        return g1.isRumbling();
    }
}