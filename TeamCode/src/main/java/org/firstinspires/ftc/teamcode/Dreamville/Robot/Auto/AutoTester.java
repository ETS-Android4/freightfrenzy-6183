package org.firstinspires.ftc.teamcode.Dreamville.Robot.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Dreamville.Robot.Auto.AutoSubsystems.AutoCarousel;
import org.firstinspires.ftc.teamcode.Dreamville.Robot.Auto.AutoSubsystems.AutoElevator;
import org.firstinspires.ftc.teamcode.Dreamville.Robot.Auto.AutoSubsystems.AutoIntake;

@TeleOp
public class AutoTester extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        AutoCarousel carousel = new AutoCarousel(hardwareMap);
        AutoElevator elevator = new AutoElevator(hardwareMap);
        AutoIntake intake = new AutoIntake(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad1.a) {
                carousel.spin();
            }

            if (gamepad1.b) {
                elevator.goToMiddle();
            }

            if (gamepad1.y) {
                intake.deposit();
            }

            if (gamepad1.x) {
                intake.stop();
            }

            carousel.update();
            elevator.update();
            intake.update();

            TelemetryPacket packet = new TelemetryPacket();

            packet.putAll(carousel.getTelemetry());
            packet.putAll(elevator.getTelemetry());
            packet.putAll(intake.getTelemetry());

            dashboard.sendTelemetryPacket(packet);
        }
    }
}