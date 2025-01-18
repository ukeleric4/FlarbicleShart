package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw {
    public Servos claw;

    public Claw(HardwareMap hw) {
        claw = new Servos(hw, "claw");
    }

    public void closeClaw() {
        claw.moveToMin();
    }

    public void openClaw() {
        claw.moveToMax();
    }

    public double getPosition() { return claw.getPosition(); }

    public void moveClawToSpecificPos(double pos) {
        claw.moveSpecificPos(pos);
    }
}
