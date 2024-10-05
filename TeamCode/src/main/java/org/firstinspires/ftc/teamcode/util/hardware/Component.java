package org.firstinspires.ftc.teamcode.util.hardware;

public abstract class Component {
    private int port;
    private String name;

    public Component(int port, String name){
        this.port = port;
        this.name = name;
    }

    public int getPort() {
        return port;
    }

    public String getName(){
        return name;
    }
}