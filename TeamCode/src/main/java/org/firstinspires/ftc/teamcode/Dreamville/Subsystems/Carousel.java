package org.firstinspires.ftc.teamcode.Dreamville.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Carousel {
    private boolean g1rb;
    private Telemetry telemetry;

    public static double carouselPower = 0.26;
    public static double brakingTime = 1;
    public static int carouselEncoder = 400;
    public static double fastTime = 0.4;

    private static int carouselStartPos = 0;

    private static DcMotor c;

    private enum carouselMode {
        BRAKING,
        IDLE,
        SLOW,
        FAST
    }

    private carouselMode carouselState = carouselMode.IDLE;

    private final ElapsedTime carouselTime = new ElapsedTime();

    public Carousel(HardwareMap hardwareMap) {
        c = hardwareMap.get(DcMotor.class, "carousel");
        c.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        c.setDirection(DcMotor.Direction.REVERSE);

        carouselStartPos = c.getCurrentPosition();
    }

    public void spin(boolean g1rb, Telemetry telemetry) {
        this.g1rb = g1rb;

        this.telemetry = telemetry;

        spinFormula();
    }

    public void spinFormula() {
        switch (carouselState) {
            case BRAKING:
                if (carouselTime.time()<brakingTime && g1rb) {
                    c.setPower(0);
                } else {
                    carouselState = carouselMode.IDLE;
                }
                break;
            case IDLE:
                c.setPower(0);
                carouselStartPos = c.getCurrentPosition();
                if (g1rb) {
                    carouselState = carouselMode.SLOW;
                }
                break;
            case SLOW:
                if (g1rb) {
                    if (Math.abs(c.getCurrentPosition()-carouselStartPos) < carouselEncoder) {
                        c.setPower(carouselPower);
                    } else {
                        carouselState = carouselMode.FAST;
                        carouselTime.reset();
                    }
                } else {
                    carouselState = carouselMode.IDLE;
                }
                break;
            case FAST:
                if (g1rb) {
                    if (carouselTime.time()<fastTime) {
                        c.setPower(1);
                    } else {
                        carouselState = carouselMode.BRAKING;
                        carouselTime.reset();
                    }
                } else {
                    carouselState = carouselMode.IDLE;
                }
                break;
        }

        telemetry.addData("carouselPower", c.getPower());
        telemetry.addData("carouselPos", c.getCurrentPosition()-carouselStartPos);
    }
}