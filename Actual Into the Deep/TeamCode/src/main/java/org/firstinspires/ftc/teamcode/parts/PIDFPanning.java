package org.firstinspires.ftc.teamcode.parts;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PIDFPanning {
    private PIDController controller;

    public static double p = 0.025, i = 0, d = 0.00000001;
    public static double f = 0;

    public int target = 0;
    double ticks_in_degree = 1993.6 / 180.0;

    public double power = 0;

    private DcMotorEx motor1;

    public PIDFPanning(HardwareMap hardwareMap) {
        controller = new PIDController(p, i, d);

        motor1 = hardwareMap.get(DcMotorEx.class, "panningmotor");
    }

    public void updatePanning() {
        controller.setPID(p, i, d);
        int motorPos = motor1.getCurrentPosition();
        double pid = controller.calculate(motorPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        power = pid + ff;
    }

    public void setTargetPos(int targetPos) {
        target = targetPos;
    }
}

