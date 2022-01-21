package org.firstinspires.ftc.teamcode.Dreamville.Robot.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Dreamville.Robot.Auto.AutoSubsystems.AutoCarousel;
import org.firstinspires.ftc.teamcode.Dreamville.Robot.Auto.AutoSubsystems.AutoElevator;
import org.firstinspires.ftc.teamcode.Dreamville.Robot.Auto.AutoSubsystems.AutoIntake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * This opmode explains how you follow multiple trajectories in succession, asynchronously. This
 * allows you to run your own logic beside the drive.update() command. This enables one to run
 * their own loops in the background such as a PID controller for a lift. We can also continuously
 * write our pose to PoseStorage.
 * <p>
 * The use of a State enum and a currentState field constitutes a "finite state machine."
 * You should understand the basics of what a state machine is prior to reading this opmode. A good
 * explanation can be found here:
 * https://www.youtube.com/watch?v=Pu7PMN5NGkQ (A finite state machine introduction tailored to FTC)
 * or here:
 * https://gm0.org/en/stable/docs/software/finite-state-machines.html (gm0's article on FSM's)
 * <p>
 * You can expand upon the FSM concept and take advantage of command based programming, subsystems,
 * state charts (for cyclical and strongly enforced states), etc. There is still a lot to do
 * to supercharge your code. This can be much cleaner by abstracting many of these things. This
 * opmode only serves as an initial starting point.
 */
@Autonomous(group = "advanced")
public class CoolMainAuto extends LinearOpMode {

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    /*
    enum State {
        TRAJECTORY_1,   // First, follow a splineTo() trajectory
        TRAJECTORY_2,   // Then, follow a lineTo() trajectory
        TURN_1,         // Then we want to do a point turn
        TRAJECTORY_3,   // Then, we follow another lineTo() trajectory
        WAIT_1,         // Then we're gonna wait a second
        TURN_2,         // Finally, we're gonna turn again
        IDLE            // Our bot will enter the IDLE state when done
    }
     */

    enum State {
        TRAJECTORY_1,
        CAROUSEL,
        TRAJECTORY_2,
        DEPOSIT_1,
        OUTTAKE_1,
        TRAJECTORY_3,
        PICKUP_1,
        TRAJECTORY_4,
        DEPOSIT_2,
        OUTTAKE_2,
        TRAJECTORY_5,
        PICKUP_2,
        TRAJECTORY_6,
        DEPOSIT_3,
        OUTTAKE_3,
        PARK,
        IDLE            // Our bot will enter the IDLE state when done
    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;

    // Define our start pose
    Pose2d startPose = new Pose2d(-23, 62.5, Math.toRadians(270));

    ElapsedTime eTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        AutoCarousel carousel = new AutoCarousel(hardwareMap);
        AutoElevator elevator = new AutoElevator(hardwareMap);
        AutoIntake intake = new AutoIntake(hardwareMap);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-54, 59, Math.toRadians(0)), Math.toRadians(180))
                .build();

        TrajectorySequence trajectory2 = drive.trajectorySequenceBuilder(trajectory1.end())
                .lineToConstantHeading(new Vector2d(-54, 24))
                .splineToConstantHeading(new Vector2d(-30, 24), Math.toRadians(0))
                .build();

        TrajectorySequence trajectory3 = drive.trajectorySequenceBuilder(trajectory2.end())
                .lineToConstantHeading(new Vector2d(-30, 16))
                .splineToConstantHeading(new Vector2d(10, 16), Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(10, 64))
                .lineToConstantHeading(new Vector2d(56, 64))
                .build();

        TrajectorySequence trajectory4 = drive.trajectorySequenceBuilder(trajectory3.end())
                .lineToConstantHeading(new Vector2d(18, 64))
                .splineToConstantHeading(new Vector2d(10, 60), Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(7, 24, Math.toRadians(180)), Math.toRadians(270))
                .build();

        TrajectorySequence trajectory5 = drive.trajectorySequenceBuilder(trajectory4.end())
                .splineToSplineHeading(new Pose2d(10, 64, Math.toRadians(0)), Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(56, 64))
                .build();

        TrajectorySequence trajectory6 = drive.trajectorySequenceBuilder(trajectory5.end())
                .lineToConstantHeading(new Vector2d(18, 64))
                .splineToConstantHeading(new Vector2d(10, 60), Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(7, 24, Math.toRadians(180)), Math.toRadians(270))
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(trajectory6.end())
                .splineToSplineHeading(new Pose2d(10, 64, Math.toRadians(0)), Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(40, 64))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        currentState = State.TRAJECTORY_1;
        drive.followTrajectorySequenceAsync(trajectory1);

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case TRAJECTORY_1:
                    if (!drive.isBusy()) {
                        currentState = State.CAROUSEL;
                        carousel.spin();
                    }
                    break;
                case CAROUSEL:
                    if (!carousel.isBusy()) {
                        currentState = State.TRAJECTORY_2;
                        drive.followTrajectorySequenceAsync(trajectory2);
                    }
                    break;
                case TRAJECTORY_2:
                    if (!drive.isBusy()) {
                        currentState = State.DEPOSIT_1;
                        elevator.goToTop();
                    }
                    break;
                case DEPOSIT_1:
                    if (!elevator.isBusy()) {
                        currentState = State.OUTTAKE_1;
                        intake.deposit();
                        eTime.reset();
                    }
                    break;
                case OUTTAKE_1:
                    if (eTime.time()>1) {
                        intake.stop();
                        currentState = State.TRAJECTORY_3;
                        drive.followTrajectorySequenceAsync(trajectory3);
                    }
                    break;
                case TRAJECTORY_3:
                    if (!drive.isBusy()) {
                        currentState = State.PICKUP_1;
                    }
                    break;
                case PICKUP_1:
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_4;
                        drive.followTrajectorySequenceAsync(trajectory4);
                    }
                    break;
                case TRAJECTORY_4:
                    if (!drive.isBusy()) {
                        currentState = State.DEPOSIT_2;
                        elevator.goToMiddle();
                    }
                    break;
                case DEPOSIT_2:
                    if (!elevator.isBusy()) {
                        currentState = State.OUTTAKE_2;
                        intake.deposit();
                        eTime.reset();
                    }
                    break;
                case OUTTAKE_2:
                    if (eTime.time()>1) {
                        intake.stop();
                        currentState = State.TRAJECTORY_5;
                        drive.followTrajectorySequenceAsync(trajectory5);
                    }
                    break;
                case TRAJECTORY_5:
                    if (!drive.isBusy()) {
                        currentState = State.PICKUP_2;
                    }
                    break;
                case PICKUP_2:
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_6;
                        drive.followTrajectorySequenceAsync(trajectory6);
                    }
                    break;
                case TRAJECTORY_6:
                    if (!drive.isBusy()) {
                        currentState = State.DEPOSIT_3;
                        elevator.goToMiddle();
                    }
                    break;
                case DEPOSIT_3:
                    if (!elevator.isBusy()) {
                        currentState = State.OUTTAKE_3;
                        intake.deposit();
                        eTime.reset();
                    }
                    break;
                case OUTTAKE_3:
                    if (eTime.time()>1) {
                        intake.stop();
                        currentState = State.PARK;
                        drive.followTrajectorySequenceAsync(park);
                    }
                    break;
                case PARK:
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }

            drive.update();
            carousel.update();
            elevator.update();
            intake.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            PoseStorage.currentPose = poseEstimate;

            TelemetryPacket packet = new TelemetryPacket();

            packet.put("x", poseEstimate.getX());
            packet.put("y", poseEstimate.getY());
            packet.put("heading", poseEstimate.getHeading());

            packet.putAll(carousel.getTelemetry());
            packet.putAll(elevator.getTelemetry());
            packet.putAll(intake.getTelemetry());

            dashboard.sendTelemetryPacket(packet);
        }
    }
}