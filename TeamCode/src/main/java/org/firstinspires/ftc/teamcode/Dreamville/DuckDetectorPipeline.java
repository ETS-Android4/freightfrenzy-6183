package org.firstinspires.ftc.teamcode.Dreamville;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class DuckDetectorPipeline extends OpenCvPipeline {

    int duckPos;

    @Override
    public Mat processFrame(Mat input)
    {
        return input;
    }

    public int getDuckPos() {
        return duckPos;
    }
}