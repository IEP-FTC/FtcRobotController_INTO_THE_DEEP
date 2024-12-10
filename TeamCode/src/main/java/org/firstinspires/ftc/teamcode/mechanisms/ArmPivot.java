package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmPivot {
    private DcMotor pivotLeft, pivotRight;

    public void init(HardwareMap HardMap) {
        pivotLeft = HardMap.get(DcMotor.class, "pivotLeft");
        pivotRight = HardMap.get(DcMotor.class, "pivotRight");
        pivotLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pivotRight.setDirection(DcMotor.Direction.REVERSE);

        pivotLeft.setTargetPosition(0);
        pivotRight.setTargetPosition(0);
        pivotLeft.setPower(0.75);
        pivotRight.setPower(0.75);
        pivotLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void moveToPosition(int degrees ) {
        int  targetPosition = ((degrees*125)/30)/(360/288);//125 = big gear, 30 = small gear, 360/288 = tick ratio
        pivotLeft.setTargetPosition(targetPosition);
        pivotRight.setTargetPosition(targetPosition);

    }
}