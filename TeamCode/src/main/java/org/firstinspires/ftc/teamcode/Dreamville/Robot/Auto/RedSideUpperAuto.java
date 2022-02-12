package org.firstinspires.ftc.teamcode.Dreamville.Robot.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Dreamville.Robot.Auto.AutoSubsystems.AutoCapper;
import org.firstinspires.ftc.teamcode.Dreamville.Robot.Auto.AutoSubsystems.AutoCarousel;
import org.firstinspires.ftc.teamcode.Dreamville.Robot.Auto.AutoSubsystems.AutoElevator;
import org.firstinspires.ftc.teamcode.Dreamville.Robot.Auto.AutoSubsystems.AutoIntake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

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
 * https://gm0.org/en/stable/docs/software/finite-state-machines.html (gm0's article on FSMs)
 * <p>
 * You can expand upon the FSM concept and take advantage of command based programming, subsystems,
 * state charts (for cyclical and strongly enforced states), etc. There is still a lot to do
 * to supercharge your code. This can be much cleaner by abstracting many of these things. This
 * opmode only serves as an initial starting point.
 */
@Autonomous(name = "redSideUpperAuto", group = "advanced")
public class RedSideUpperAuto extends LinearOpMode {
    OpenCvCamera camera;
    DuckDetectorPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    double cx = 456.81749;
    double cy = 373.69443;
    double fx = 1049.85492;
    double fy = 1051.06561;

    int tagPos = 0;

    // UNITS ARE METERS
    double tagSize = 0.075;

    int ID_TAG_OF_INTEREST = 1; // Tag ID 18 from the 36h11 family

    AprilTagDetection tagOfInterest = null;

    enum State {
        TRAJECTORY_1,
        CAROUSEL,
        TRAJECTORY_2,
        DEPOSIT_1,
        OUTTAKE_1,
        TRAJECTORY_3,
        PICKUP_1,
        DEPOSIT_2,
        OUTTAKE_2,
        TRAJECTORY_5,
        PICKUP_2,
        DEPOSIT_3,
        OUTTAKE_3,
        PARK,
        IDLE            // Our bot will enter the IDLE state when done
    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;

    // Define our start pose
    Pose2d startPose = new Pose2d(11.75, -62.5, Math.toRadians(90));

    ElapsedTime eTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new DuckDetectorPipeline(tagSize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
                dashboard.startCameraStream(camera, 0);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        AutoCarousel carousel = new AutoCarousel(hardwareMap, 1);
        AutoElevator elevator = new AutoElevator(hardwareMap);
        AutoIntake intake = new AutoIntake(hardwareMap);
        AutoCapper capper = new AutoCapper(hardwareMap);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-54, -57), Math.toRadians(180))
                .build();

        while (!isStarted() && !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == ID_TAG_OF_INTEREST) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    packet.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest, packet);
                } else {
                    packet.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        packet.addLine("(The tag has never been seen)");
                    } else {
                        packet.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest, packet);
                    }
                }

            } else {
                packet.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    packet.addLine("(The tag has never been seen)");
                } else {
                    packet.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest, packet);
                }

            }

            dashboard.sendTelemetryPacket(packet);
        }

        if (isStopRequested()) return;

        camera.pauseViewport();
        if (tagOfInterest == null) {
            tagPos = 1;
        } else {
            if (tagOfInterest.pose.x > 0) {
                tagPos = 2;
            } else if (tagOfInterest.pose.x < 0) {
                tagPos = 3;
            }
        }

        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        currentState = State.TRAJECTORY_1;
        drive.followTrajectoryAsync(trajectory1);

        while (opModeIsActive() && !isStopRequested()) {
            Pose2d poseEstimate = drive.getPoseEstimate();
            PoseStorage.currentPose = poseEstimate;
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
                        TrajectorySequence trajectory2 = drive.trajectorySequenceBuilder(poseEstimate)
                                .lineToLinearHeading(new Pose2d(-8, -44, Math.toRadians(90)))
                                .build();
                        drive.followTrajectorySequenceAsync(trajectory2);
                    }
                    break;
                case TRAJECTORY_2:
                    if (!drive.isBusy()) {
                        currentState = State.DEPOSIT_1;
                        if (tagPos == 1) {
                            elevator.goToBottom();
                        } else if (tagPos == 2) {
                            elevator.goToMiddle();
                        } else {
                            elevator.goToTop();
                        }
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
                    if (!intake.isBusy()) {
                        currentState = State.TRAJECTORY_3;
                        TrajectorySequence trajectory3 = drive.trajectorySequenceBuilder(poseEstimate)
                                .lineToLinearHeading(new Pose2d(10, -65, Math.toRadians(0)))
                                .build();
                        drive.followTrajectorySequenceAsync(trajectory3);
                        elevator.goToGround();
                    }
                    break;
                case TRAJECTORY_3:
                    if (!drive.isBusy()) {
                        currentState = State.PICKUP_1;
                        TrajectorySequence pickup = drive.trajectorySequenceBuilder(poseEstimate)
                                .lineToConstantHeading(new Vector2d(70, -65))
                                .build();
                        drive.followTrajectorySequenceAsync(pickup);
                        intake.intake();
                    }
                    break;
                case PICKUP_1:
                    if (!intake.isBusy()) {
                        currentState = State.DEPOSIT_2;
                        drive.breakFollowing();
                        TrajectorySequence trajectory4 = drive.trajectorySequenceBuilder(poseEstimate)
                                .lineToConstantHeading(new Vector2d(10, -65))
                                .splineToSplineHeading(new Pose2d(-8, -44, Math.toRadians(90)), Math.toRadians(180))
                                .build();
                        drive.followTrajectorySequenceAsync(trajectory4);
                        elevator.goToMiddle();
                    }
                    break;
                case DEPOSIT_2:
                    if (!elevator.isBusy() && !drive.isBusy()) {
                        currentState = State.OUTTAKE_2;
                        intake.deposit();
                    }
                    break;
                case OUTTAKE_2:
                    if (!intake.isBusy()) {
                        currentState = State.TRAJECTORY_5;
                        TrajectorySequence trajectory5 = drive.trajectorySequenceBuilder(poseEstimate)
                                .lineToLinearHeading(new Pose2d(10, -65, Math.toRadians(0)))
                                .lineToConstantHeading(new Vector2d(70, -65))
                                .build();
                        drive.followTrajectorySequenceAsync(trajectory5);
                        elevator.goToGround();
                        intake.intake();
                    }
                    break;
                case PICKUP_2:
                    if (!intake.isBusy()) {
                        currentState = State.DEPOSIT_3;
                        drive.breakFollowing();
                        TrajectorySequence trajectory6 = drive.trajectorySequenceBuilder(poseEstimate)
                                .lineToConstantHeading(new Vector2d(10, -65))
                                .splineToSplineHeading(new Pose2d(0, -44, Math.toRadians(90)), Math.toRadians(180))
                                .build();
                        drive.followTrajectorySequenceAsync(trajectory6);
                        elevator.goToMiddle();
                    }
                    break;
                case DEPOSIT_3:
                    if (!elevator.isBusy() && !drive.isBusy()) {
                        currentState = State.OUTTAKE_3;
                        intake.deposit();
                    }
                    break;
                case OUTTAKE_3:
                    if (!intake.isBusy()) {
                        currentState = State.PARK;
                        TrajectorySequence park = drive.trajectorySequenceBuilder(poseEstimate)
                                .lineToLinearHeading(new Pose2d(10, -65, Math.toRadians(0)))
                                .lineToConstantHeading(new Vector2d(70, -65))
                                .build();
                        drive.followTrajectorySequenceAsync(park);
                        elevator.goToBottom();
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
            capper.update();

            TelemetryPacket packet = drive.getPacket();

            packet.put("tagPos", tagPos);

            packet.putAll(carousel.getTelemetry());
            packet.putAll(elevator.getTelemetry());
            packet.putAll(intake.getTelemetry());

            dashboard.sendTelemetryPacket(packet);
        }
    }

    void tagToTelemetry(AprilTagDetection detection, TelemetryPacket packet) {
        packet.addLine(String.format("\nDetected tag ID=%d", detection.id));
        packet.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        packet.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        packet.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        packet.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        packet.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        packet.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}