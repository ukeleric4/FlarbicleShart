package org.firstinspires.ftc.teamcode.pedroPathing.localization.localizers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Encoder Telemetry", group = "Telemetry")
@Disabled
public class EncoderTelemetry2 extends OpMode {

    // Declare encoder variables
    private DcMotorEx perpendicularEncoder;
    private DcMotorEx horizontalEncoder;

    @Override
    public void init() {
        // Initialize hardware map
        perpendicularEncoder = hardwareMap.get(DcMotorEx.class, "bl");
        horizontalEncoder = hardwareMap.get(DcMotorEx.class, "br");

        // Ensure encoders are found
        if (perpendicularEncoder == null || horizontalEncoder == null) {
            telemetry.addLine("Error: Check hardware configuration names!");
            telemetry.update();
            return;
        }

        // Reset encoder values
        perpendicularEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        horizontalEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Set to run without encoder mode
        perpendicularEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        horizontalEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Initialization complete!");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Get encoder positions
        int perpendicularPosition = perpendicularEncoder.getCurrentPosition();
        int horizontalPosition = horizontalEncoder.getCurrentPosition();

        // Output encoder values to telemetry
        telemetry.addData("Perpendicular Encoder Position", perpendicularPosition);
        telemetry.addData("Horizontal Encoder Position", horizontalPosition);
        telemetry.update();
    }
}