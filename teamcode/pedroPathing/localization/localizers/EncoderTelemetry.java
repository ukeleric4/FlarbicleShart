package org.firstinspires.ftc.teamcode.pedroPathing.localization.localizers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Encoder Telemetry with Movement", group="Telemetry")
@Disabled
public class EncoderTelemetry extends OpMode {

    // Motor declarations
    private DcMotor fl, fr, bl, br;

    @Override
    public void init() {
        // Initialize motors from the hardware map
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Get and send encoder positions to telemetry
        telemetry.addData("Forward Encoder", br.getCurrentPosition());
        telemetry.addData("Strafe Encoder", bl.getCurrentPosition());
        telemetry.update();
    }

    /**
     * Method to move all motors forward at a given power
     * @param power The power level to set the motors (range: -1.0 to 1.0)
     */
    private void moveForward(double power) {
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }
}
