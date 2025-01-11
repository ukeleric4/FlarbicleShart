package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Panning {
    public DcMotor panningMotor;

    public Panning(HardwareMap hardwareMap) {
        panningMotor = hardwareMap.get(DcMotor.class, "panningmotor");
        panningMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void runUp() {
        panningMotor.setPower(-1.0);
    }

    public void runDown() {
        panningMotor.setPower(1.0);
    }

    public void stop() {
        panningMotor.setPower(0);
    }
}
