package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanisms.CameraStreamProcessor;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Disabled
@Autonomous
public class BennettAuto extends OpMode {
    AprilTagProcessor aprilTagProcessor;
    VisionPortal visionPortal;
    MecanumDrive drive = new MecanumDrive();
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive.init(hardwareMap);
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
            drive.stop();
        } else {
            // Loop through all detected tags
            for (AprilTagDetection detection : currentDetections) {
                idsFound.append(detection.id).append(" ");

                AprilTagPoseFtc pose = detection.ftcPose;

                // Report the pose (translation and rotation) of the detected tag

                // figure out how to use the range and bearing to make it work no matter the distance
                // that way it can go to a distance from the tag, then do stuff
                /* probably needs variables to get mecanumDriveAuto.drive(1, 0, 0) into mecanumDriveAuto.drive(?, ?, ?)
                need to do math, target distance 3 ft away, and do math to get to 36 inches no matter where
                how about: forward = range  - 36 check check!!!!
                right =
                or use the code below then move to the left and then forward to end position
                drive.goToBearingAndRange(pose.bearing, correctRange(pose.range)); and subtract 36 from range
                 */
                if (pose != null) {
                    drive.goToBearingAndRange(pose.bearing, correctRange(pose.range));
                    /* switch the goToBearingAndRange to something that works for mecanum wheels which is the Bearing and Range AND Yaw
                    add a
                    AprilTagDetection myAprilTagDetection;
                     int myAprilTagID;
                     myAprilTagID = myAprilTagDetection.metadata.ID;
                     then if ID = 1 do that, if ID = 2 do this, etc
                    if (myAprilTadID = 1) {
                     move to 2 ft from the tag and then go around chair to position B
                     } else if (myAprilTadID = 2) {
                     somecodehere
                     } else if etc
                     */

                    /* mecanumDriveAuto.drive(1, 0, 0);
                    try {
                        Thread.sleep(750);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    */
                    // how do I get the IDs for the April tags?

                    mecanumDriveAuto.drive(0, 1, 0);
                    try {
                        Thread.sleep(1500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    MecanumDriveAuto.drive(-1, 0, 0);
                    try {
                        Thread.sleep(750);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }

                } else {
                    telemetry.addLine("Pose data not available for Tag ID: " + detection.id);
                    drive.stop();
                }
            }
        }



        /*if (!doneOnce) {
            mecanumDriveAuto.drive(1, 0, 0);
            try {
                Thread.sleep(750);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            mecanumDriveAuto.drive(0, 1, 0);
            try {
                Thread.sleep(1500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            mecanumDriveAuto.drive(-1, 0, 0);
            try {
                Thread.sleep(750);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            doneOnce = true;
        }

        mecanumDriveAuto.drive(0,0,0); */


    }

}