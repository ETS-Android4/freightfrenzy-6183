package org.firstinspires.ftc.teamcode.Dreamville.Robot.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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

@Autonomous(group = "advanced", preselectTeleOp = "MainTeleOp")
public class BruhMainAuto extends LinearOpMode {
    OpenCvCamera camera;
    DuckDetectorPipeline aprilTagDetectionPipeline;

    private static BNO055IMU imu;

    static final double FEET_PER_METER = 3.28084;

    double cx = 456.81749;
    double cy = 373.69443;
    double fx = 1049.85492;
    double fy = 1051.06561;

    int tagPos = 0;

    // UNITS ARE METERS
    double tagsize = 0.1;

    int ID_TAG_OF_INTEREST = 1; // Tag ID 18 from the 36h11 family

    AprilTagDetection tagOfInterest = null;

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
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample op mode
        parameters.mode = BNO055IMU.SensorMode.IMU;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        imu.startAccelerationIntegration(null, null, 1000);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new DuckDetectorPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(960, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        AutoCarousel carousel = new AutoCarousel(hardwareMap, -1);
        AutoElevator elevator = new AutoElevator(hardwareMap);
        AutoIntake intake = new AutoIntake(hardwareMap);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-55, 54, Math.toRadians(0)), Math.toRadians(180))
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
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            dashboard.sendTelemetryPacket(packet);
            sleep(20);
        }

        if (isStopRequested()) return;

        if (tagOfInterest == null) {
            tagPos = 3;
        } else {
            if (tagOfInterest.pose.x > 0) {
                tagPos = 2;
            } else if (tagOfInterest.pose.x < 0) {
                tagPos = 1;
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
                        currentState = State.PARK;
                        TrajectorySequence trajectory2 = drive.trajectorySequenceBuilder(poseEstimate)
                                .lineToConstantHeading(new Vector2d(-54, 30))
                                .build();
                        drive.followTrajectorySequenceAsync(trajectory2);
                    }
                    break;
                case PARK:
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                        elevator.goToGround();
                    }
                    break;
                case IDLE:
                    break;
            }

            PoseStorage.currentHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            drive.update();
            carousel.update();
            elevator.update();
            intake.update();

            TelemetryPacket packet = new TelemetryPacket();

            packet.put("imuRead", PoseStorage.currentHeading);
            packet.put("x", poseEstimate.getX());
            packet.put("y", poseEstimate.getY());
            packet.put("heading", poseEstimate.getHeading());
            packet.put("tagPos", tagPos);

            packet.putAll(carousel.getTelemetry());
            packet.putAll(elevator.getTelemetry());
            packet.putAll(intake.getTelemetry());

            dashboard.sendTelemetryPacket(packet);
        }
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}