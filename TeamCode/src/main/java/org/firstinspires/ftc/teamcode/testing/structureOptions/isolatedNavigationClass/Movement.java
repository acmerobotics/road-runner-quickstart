package org.firstinspires.ftc.teamcode.testing.structureOptions.isolatedNavigationClass;

// Represents a translational movement followed by a rotational movement.
public class Movement {
    private double side;
    private double forward;
    private double rot;

    Movement(double side, double forward, double rot){
        set(side, forward, rot);
    }
    Movement(Movement position){
        set(position);
    }
    Movement(double side, double forward){
        this(side, forward, 0);
    }
    Movement(){
        this(0, 0);
    }

    void set(double side, double forward, double rot){
        this.side = side;
        this.forward = forward;
        this.rot = rot;
    }
    void set(Movement pos){
        set(
                pos.getSide(),
                pos.getForward(),
                pos.getRot()
        );
    }
    void set(double side, double forward){
        this.side = side;
        this.forward = forward;
    }
    void setSide(double side){
        this.side = side;
    }
    void setForward(double forward){
        this.forward = forward;
    }
    void setRot(double rot){
        this.rot = rot;
    }

    double getSide(){
        return side;
    }
    double getForward(){
        return forward;
    }
    double getRot(){
        return rot;
    }

    void add(double side, double forward, double rot){
        this.side += side*Math.cos(this.rot) + forward*Math.cos(this.rot + Math.PI/2);
        this.forward += side*Math.sin(this.rot) + forward*Math.sin(this.rot + Math.PI/2);
        this.rot += rot;
    }
    void add(Movement movement){
        add(
                movement.getSide(),
                movement.getForward(),
                movement.getRot()
        );
    }
    void subtract(double side, double forward, double rot){
        this.rot -= rot;
        this.side -= side*Math.cos(this.rot) + forward*Math.cos(this.rot + Math.PI/2);
        this.forward -= side*Math.sin(this.rot) + forward*Math.sin(this.rot + Math.PI/2);
    }
}
