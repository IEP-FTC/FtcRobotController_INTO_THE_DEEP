package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanisms.CameraStreamProcessor;
import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
public class DriveToAprilTags extends OpMode {

    AprilTagProcessor aprilTagProcessor;
    VisionPortal visionPortal;
    ProgrammingBot bot = new ProgrammingBot();
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        bot.init(hardwareMap);
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        //visionPortal = VisionPortal.easyCreateWithDefaults(webcamName, aprilTagProcessor);

        CameraStreamProcessor processor = new CameraStreamProcessor();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(processor)
                .addProcessor(aprilTagProcessor)
                .setCamera(BuiltinCameraDirection.BACK)
                .build();

        FtcDashboard.getInstance().startCameraStream(processor, 0);

    }

    @Override
    public void loop() {
        telemetry.addLine("Status: Looking for Tags");
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        StringBuilder idsFound = new StringBuilder();
        if (currentDetections.isEmpty()) {
            telemetry.addLine("No AprilTags detected.");
            bot.slowDown();
        } else {
            // Loop through all detected tags
            for (AprilTagDetection detection : currentDetections) {
                idsFound.append(detection.id).append(" ");

                AprilTagPoseFtc pose = detection.ftcPose;

                // Report the pose (translation and rotation) of the detected tag
                if (pose != null) {
                    telemetry.addLine("Detected Tag ID: " + detection.id);
                    telemetry.addData("Range:", "%.2f in", correctRange(pose.range));
                    telemetry.addData("Bearing:", "%.2f deg", pose.bearing);
                    bot.goToBearingAndRange(pose.bearing, correctRange(pose.range));
                } else {
                    telemetry.addLine("Pose data not available for Tag ID: " + detection.id);
                    bot.slowDown();
                }
            }
        }
        // Display all detected tag IDs
        telemetry.addData("April Tags", idsFound.toString());
        telemetry.update();

    }

    public double correctRange(double range){
        return range*1.6; // This correction is based on measurements we made.

    }

}