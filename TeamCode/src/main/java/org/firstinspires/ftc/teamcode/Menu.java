package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

/*
 * A class for creating joystick-controlled menus usable during the initialization period of autonomous.
 * Typical use case within a LinearOpMode:

   public void runOpMode() {
    Menu myMenu = new Menu(this);
    myMenu.add(new MenuItem(0, "Delay Time", 0, 5, 0.25)); // default value, label, min, max, increment
    while (!isStarted) {
        myMenu.update();
        myMenu.display();
    }
}
*/

public class Menu {

    LinearOpMode parent;
    ArrayList<MenuItem> items = new ArrayList<>();
    int menuActiveItem = 0;
    boolean buttonPressed = false;

    public Menu(LinearOpMode p) {
        parent = p;
    }

    public void add(MenuItem m) {
        items.add(m);
    }

    public void display() {
        int i;
        for (i=0; i<items.size(); i++) {
            if (i == menuActiveItem) {
                parent.telemetry.addLine(">"+items.get(i).getRepr());
            }
            else {
                parent.telemetry.addLine(items.get(i).getRepr());
            }
        }
        //parent.telemetry.update();
    }

    public void update() {
        double joystickThreshold = 0.20;
        int delay = 250;
        if ((parent.gamepad1.right_stick_y > joystickThreshold)||parent.gamepad1.a) {
            items.get(menuActiveItem).down();
            parent.sleep(delay);
        }
        else if((parent.gamepad1.right_stick_y < -joystickThreshold)||parent.gamepad1.y) {
            items.get(menuActiveItem).up();
            parent.sleep(delay);
        }
        else if(parent.gamepad1.b && !buttonPressed) {
            menuActiveItem++;
            if (menuActiveItem >= items.size()) {
                menuActiveItem = 0;
            }
            buttonPressed = true;
            parent.sleep(delay);
        }
        else if(parent.gamepad1.x && !buttonPressed) {
            menuActiveItem--;
            if (menuActiveItem < 0) {
                menuActiveItem = items.size() - 1;
            }
            buttonPressed = true;
            parent.sleep(delay);
        }
        else {
            buttonPressed = false;
        }
    }

    public double get(int index) {
        return items.get(index).getValue();
    }
}