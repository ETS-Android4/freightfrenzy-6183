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

        AutoCarousel carousel = new AutoCarousel(hardwareMap, -1);
        AutoElevator elevator = new AutoElevator(hardwareMap);
        AutoIntake intake = new AutoIntake(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad1.a) {
                elevator.goToTop();
            }

            if (gamepad1.b) {
                elevator.goToMiddle();
            }

            if (gamepad1.y) {
                elevator.goToBottom();
            }

            if (gamepad1.x) {
                elevator.goToGround();
            }

            if (gamepad1.right_bumper) {
                intake.intake();
            }

            if (gamepad1.left_bumper) {
                intake.deposit();
            }

            if (gamepad1.dpad_left) {
                intake.stop();
            }

            carousel.update();
            elevator.update();
            intake.update();

            TelemetryPacket packet = new TelemetryPacket();

            packet.putAll(carousel.getTelemetry());
            packet.putAll(elevator.getTelemetry());
            packet.putAll(intake.getTelemetry());

            packet.put("elevatorBusy", elevator.isBusy());
            packet.put("carouselBusy", carousel.isBusy());
            packet.put("intakeBusy", intake.isBusy());

            dashboard.sendTelemetryPacket(packet);
        }
    }
}