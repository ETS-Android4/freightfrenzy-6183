package org.firstinspires.ftc.teamcode.Dreamville.Robot.Auto.AutoSubsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.LinkedHashMap;
import java.util.Map;

@Config
public class AutoIntake {
    private static final Map<String, Object> telemetry = new LinkedHashMap<>();

    private static DcMotor intake;
    private static RevColorSensorV3 colorSensor;
    public static double ltDivisor = 2;
    public static double stuckPower = 0.20;
    public static double stuckTime = 0.5;
    private static boolean stuck = false;

    private enum intakeMode {
        IDLE,
        INTAKE,
        DEPOSIT,
        STUCK,
        CLEAR,
        WAIT
    }

    private intakeMode intakeState = intakeMode.IDLE;

    private static ElapsedTime eTime = new ElapsedTime();

    public AutoIntake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "sensor_color");
    }

    public void update() {
        double distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);

        telemetry.put("intakeDistance", distance);
        telemetry.put("stuck", stuck);

        switch (intakeState) {
            case IDLE:
                intake.setPower(0);
                break;
            case INTAKE:
                intake.setPower(1);

                if (distance < 4.2) {
                    eTime.reset();
                    intakeState = intakeMode.WAIT;
                }
                break;
            case DEPOSIT:
                if (distance > 4.5) {
                    stuck = true;
                    eTime.reset();
                    intakeState = intakeMode.STUCK;
                } else {
                    intake.setPower(-1 / ltDivisor);

                    if (distance > 4.5) {
                        intakeState = intakeMode.IDLE;
                    }
                }
                break;
            case STUCK:
                intake.setPower(-stuckPower);
                if (distance < 4.5 || eTime.time() > stuckTime) {
                    intakeState = intakeMode.CLEAR;
                }
                break;
            case CLEAR:
                intake.setPower(-1 / ltDivisor);
                if (distance > 4.5) {
                    stuck = false;
                    intakeState = intakeMode.IDLE;
                }
                break;
            case WAIT:
                if (eTime.seconds() > 0.1) {
                    intakeState = intakeMode.IDLE;
                }
        }
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