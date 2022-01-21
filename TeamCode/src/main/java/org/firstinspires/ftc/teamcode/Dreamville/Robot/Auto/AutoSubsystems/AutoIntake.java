package org.firstinspires.ftc.teamcode.Dreamville.Robot.Auto.AutoSubsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.LinkedHashMap;
import java.util.Map;

@Config
public class AutoIntake {
    private static final Map<String, Object> telemetry = new LinkedHashMap<>();

    private static DcMotor intake;
    private static double pwr = 0;

    private enum intakeMode {
        IDLE,
        INTAKE,
        DEPOSIT
    }

    private intakeMode intakeState = intakeMode.IDLE;

    public AutoIntake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
    }

    public void update() {
        switch (intakeState) {
            case IDLE:
                pwr = 0;
                break;
            case INTAKE:
                pwr = 1;
                break;
            case DEPOSIT:
                pwr = -0.5;
                break;
        }

        intake.setPower(pwr);

        telemetry.put("intake speed", pwr);
    }

    public void stop() {
        intakeState = intakeMode.IDLE;
    }

    public void intake() {
        intakeState = intakeMode.INTAKE;
    }

    public void deposit() {
        intakeState = intakeMode.DEPOSIT;
    }

    public boolean isBusy() {
        return intakeState != intakeMode.IDLE;
    }

    public Map<String,Object> getTelemetry() {
        return telemetry;
    }
}