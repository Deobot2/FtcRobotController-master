package org.firstinspires.ftc.teamcode.mypackage.Pathfinder;

public class Pathfinder {
    private Dcmotor fr;
    private Dcmotor fl;
    private Dcmotor br;
    private Dcmotor bl;
    public Pathfinder(Dcmotor fr, Dcmotor fl, Dcmotor br, Dcmotor bl) {
        this.fr = fr;
        this.fl = fl;
        this.br = br;
        this.bl = bl;
    }

    public void reverseMotor(Dcmotor motor) {
        motor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void Drive() {
        
    }
}
