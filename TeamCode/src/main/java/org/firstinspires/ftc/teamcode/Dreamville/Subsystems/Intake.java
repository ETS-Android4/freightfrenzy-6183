package org.firstinspires.ftc.teamcode.Dreamville.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Intake {
    public static boolean intakeForward = true;

    private double g1rt, g1lt;
    private Telemetry telemetry;

    private DcMotor intake;

    public void init(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
    }

    public void intake(double g1rt, double g1lt, Telemetry telemetry) {
        this.g1rt = g1rt;
        this.g1lt = g1lt;

        this.telemetry = telemetry;

        controls();
    }

    public void controls() {
        if (intakeForward) {
            intake.setDirection(DcMotor.Direction.FORWARD);
            intake.setPower(g1rt-(g1lt/2));
        } else {
            intake.setDirection(DcMotor.Direction.REVERSE);
            intake.setPower(g1rt-(g1lt/2));
        }
    }
}