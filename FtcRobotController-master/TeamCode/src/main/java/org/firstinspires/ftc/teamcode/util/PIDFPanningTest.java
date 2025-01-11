package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
@TeleOp
public class PIDFPanningTest extends LinearOpMode {
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;
    double ticks_in_degree = 1993.6 / 180.0;

    private DcMotorEx motor1;

    public void setTargetPos(int targetPos) {
        target = targetPos;
    }

    public int getTargetPos() {
        return target;
    }

    public void runForward() {
        target += 5;
    }

    public void runBackward() {
        if (target >= 20) {
            target -= 5;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        controller = new PIDController(p, i, d);

        motor1 = hardwareMap.get(DcMotorEx.class, "panningmotor");

        while (opModeIsActive()) {
            controller.setPID(p, i, d);
            int motorPos = -(motor1.getCurrentPosition());
            double pid = controller.calculate(motorPos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

            double power = pid + ff;

            motor1.setPower(power);
        }
    }
}

