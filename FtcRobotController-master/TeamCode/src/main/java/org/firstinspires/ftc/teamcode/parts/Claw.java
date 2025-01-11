package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw {
    public Servos left;
    public Servos right;

    public Claw(HardwareMap hw) {
        left = new Servos(hw, "left");
        right = new Servos(hw, "right");
    }

    public void closeClaw() {
        left.moveSpecificPos(0.3);
        right.moveSpecificPos(0.7);
    }

    public void openClaw() {
        left.moveSpecificPos(0.6);
        right.moveSpecificPos(0.4);
    }

    public void moveClawToSpecificPos(double pos) {
        left.moveSpecificPos(pos);
        right.moveSpecificPos(pos);
    }
}
