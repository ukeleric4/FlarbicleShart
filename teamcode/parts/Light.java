package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Light {
    public Servo light;

    public Light(HardwareMap hw) { light = hw.get(Servo.class, "light"); }


    public void goToRed() { light.setPosition(0.279); }

    public void goToOrange() { light.setPosition(0.333); }

    public void goToYellow() { light.setPosition(0.388); }

    public void goToSage() { light.setPosition(0.444); }

    public void goToGreen() { light.setPosition(0.5); }

    public void goToAzure() { light.setPosition(0.555); }

    public void goToBlue() { light.setPosition(0.611); }

    public void goToIndigo() { light.setPosition(0.666); }

    public void goToViolet() { light.setPosition(0.722); }

    public void turnOff() { light.setPosition(0); }

    public void goToWhite() { light.setPosition(1); }

    public void goLight(double pos) { light.setPosition(pos); }
}
