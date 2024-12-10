package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanisms.CameraStreamProcessor;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
public class DriveAroundChairOpMode extends OpMode {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    MecanumDrive mecanumDrive = new MecanumDrive();
    private double tag13Range = 0.0;
    private double tag11Range = 0.0;
    public enum AutoSteps{
        BACK_TO_RANGE,
        STRAFE_LEFT,
        FORWARD,
        STOP
    }

    private AutoSteps step = AutoSteps.BACK_TO_RANGE;

    @Override
    public void init() {
        mecanumDrive.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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

    public void init_loop() {

        telemetry.addLine("Status: Looking for Tags");
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        StringBuilder idsFound = new StringBuilder();
        if (currentDetections.isEmpty()) {
            telemetry.addLine("No AprilTags detected.");
        } else {
            // Loop through all detected tags
            for (AprilTagDetection detection : currentDetections) {
                idsFound.append(detection.id).append(" ");

                AprilTagPoseFtc pose = detection.ftcPose;

                // Report the pose (translation and rotation) of the detected tag
                if (pose != null) {

                    if (detection.id == 13) {
                        tag13Range = correctRange(pose.range);
                    }

                } else {
                    telemetry.addLine("Pose data not available for Tag ID: " + detection.id);
                }
            }
        }
        // Display all detected tag IDs
        telemetry.addData("April Tags", idsFound.toString());
        telemetry.addData("Tag 13 Range", tag13Range);
        telemetry.addData("Tag 11 Range", tag11Range);
        telemetry.update();
    }

    @Override
    public void loop() {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        StringBuilder idsFound = new StringBuilder();

        if (currentDetections.isEmpty()) {
            telemetry.addLine("No AprilTags detected.");
            tag11Range = 0;
        } else {
            // Loop through all detected tags
            for (AprilTagDetection detection : currentDetections) {
                idsFound.append(detection.id).append(" ");

                AprilTagPoseFtc pose = detection.ftcPose;

                // Report the pose (translation and rotation) of the detected tag
                if (pose != null) {

                    if (detection.id == 13) {
                        tag13Range = correctRange(pose.range);
                    }
                    if (detection.id == 11) {
                        tag11Range = correctRange(pose.range);
                    }

                } else {
                    telemetry.addLine("Pose data not available for Tag ID: " + detection.id);
                }
            }
        }
        telemetry.addData("April Tags", idsFound.toString());
        telemetry.addData("Tag 13 Range", tag13Range);
        telemetry.addData("Tag 11 Range", tag11Range);
        telemetry.update();
        switch(step){
            case BACK_TO_RANGE:
                mecanumDrive.drive(-.50,0,0);
                if(tag13Range>40){
                    step = AutoSteps.STRAFE_LEFT;
                    mecanumDrive.drive(0,0,0);
                }
                break;
            case STRAFE_LEFT:
                mecanumDrive.driveGyroCorrected(0,-.3 ,0);
                if(tag11Range>1){
                    step = AutoSteps.FORWARD;
                    mecanumDrive.drive(0,0,0);
                }
                break;
            case FORWARD:
                mecanumDrive.drive(.25,0,0);
                if(tag11Range<10){
                    step = AutoSteps.STOP;
                }
                break;

            case STOP:
                mecanumDrive.drive(0,0,0);
                break;

        }

    }

    public double correctRange(double range){
        return range*1.6; // This correction is based on measurements we made.

    }
}
