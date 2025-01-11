package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Servos {
    public Servo servo;

    static final double MAX_POS = 1.0;
    static final double MIN_POS = 0.0;

    public Servos(HardwareMap hw, String name) { servo = hw.get(Servo.class, name); }

    public void moveToMax() {
        servo.setPosition(MAX_POS);
    }

    public void moveToMin() { servo.setPosition(MIN_POS); }

    public void moveSpecificPos(double pos) { servo.setPosition(pos); }
}
