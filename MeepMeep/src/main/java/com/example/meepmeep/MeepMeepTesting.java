package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(12.83465, 16.92913)
                .setConstraints(52.48291908330528, 52.48291908330528, Math.toRadians(300.704976), Math.toRadians(300.704976), 12.3)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-23, 62.5, Math.toRadians(270)))
                                .splineToLinearHeading(new Pose2d(-54, 59, Math.toRadians(0)), Math.toRadians(180))
                                .addTemporalMarker(() -> {
                                    //autoSpinCarousel();
                                })
                                .lineToConstantHeading(new Vector2d(-54, 24))
                                .splineToConstantHeading(new Vector2d(-30, 24), Math.toRadians(0))
                                .addTemporalMarker(() -> {
                                    //depositFreight();
                                })
                                .lineToConstantHeading(new Vector2d(-30, 16))
                                .splineToConstantHeading(new Vector2d(10, 16), Math.toRadians(90))
                                .lineToConstantHeading(new Vector2d(10, 64))
                                .lineToConstantHeading(new Vector2d(56, 64))
                                .addTemporalMarker(() -> {
                                    //pickupFreight();
                                })
                                .lineToConstantHeading(new Vector2d(18, 64))
                                .splineToConstantHeading(new Vector2d(10, 60), Math.toRadians(270))
                                .splineToSplineHeading(new Pose2d(7, 24, Math.toRadians(180)), Math.toRadians(270))
                                .addTemporalMarker(() -> {
                                    //depositFreight();
                                })
                                .splineToSplineHeading(new Pose2d(10, 64, Math.toRadians(0)), Math.toRadians(90))
                                .lineToConstantHeading(new Vector2d(56, 64))
                                .addTemporalMarker(() -> {
                                    //pickupFreight();
                                })
                                .lineToConstantHeading(new Vector2d(18, 64))
                                .splineToConstantHeading(new Vector2d(10, 60), Math.toRadians(270))
                                .splineToSplineHeading(new Pose2d(7, 24, Math.toRadians(180)), Math.toRadians(270))
                                .addTemporalMarker(() -> {
                                    //depositFreight();
                                })
                                .splineToSplineHeading(new Pose2d(10, 64, Math.toRadians(0)), Math.toRadians(90))
                                .lineToConstantHeading(new Vector2d(40, 64))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}