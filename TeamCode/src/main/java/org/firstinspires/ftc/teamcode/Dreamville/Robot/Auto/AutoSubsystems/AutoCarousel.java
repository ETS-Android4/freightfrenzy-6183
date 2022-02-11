package org.firstinspires.ftc.teamcode.Dreamville.Robot.Auto.AutoSubsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.LinkedHashMap;
import java.util.Map;

@Config
public class AutoCarousel {
    private static final Map<String, Object> telemetry = new LinkedHashMap<>();

    public static double carouselPower = 0.26;
    public static int carouselEncoder = 400;
    public static double fastTime = 0.4;

    private static int carouselStartPos = 0;
    private static int carouselFlipper = -1;

    private static DcMotor c;

    private enum carouselMode {
        IDLE,
        INIT,
        SLOW,
        FAST
    }

    private static carouselMode carouselState = carouselMode.IDLE;

    private static final ElapsedTime carouselTime = new ElapsedTime();

    public AutoCarousel(HardwareMap hardwareMap, int flipper) {
        c = hardwareMap.get(DcMotor.class, "carousel");
        c.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        c.setDirection(DcMotor.Direction.REVERSE);

        carouselFlipper = flipper;

        carouselStartPos = c.getCurrentPosition();
    }

    public void update() {
        switch (carouselState) {
            case IDLE:
                break;
            case INIT:
                carouselStartPos = c.getCurrentPosition();
                carouselState = carouselMode.SLOW;
                break;
            case SLOW:
                if (Math.abs(c.getCurrentPosition()-carouselStartPos) < carouselEncoder) {
                    c.setPower(carouselPower * carouselFlipper);
                } else {
                    carouselState = carouselMode.FAST;
                    carouselTime.reset();
                }
                break;
            case FAST:
                if (carouselTime.time()<fastTime) {
                    c.setPower(carouselFlipper);
                } else {
                    c.setPower(0);
                    carouselState = carouselMode.IDLE;
                }
                break;
        }

        telemetry.put("carouselPower", c.getPower());
        telemetry.put("carouselPos", c.getCurrentPosition()-carouselStartPos);
    }

    public void spin() {
        carouselState = carouselMode.INIT;
    }

    public boolean isBusy() {
        return carouselState != carouselMode.IDLE;
    }

    public Map<String,Object> getTelemetry() {
        return telemetry;
    }
}