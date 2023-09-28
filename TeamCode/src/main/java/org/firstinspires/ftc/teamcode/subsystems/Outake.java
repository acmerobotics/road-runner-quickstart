package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Mechanism;

import java.util.ArrayList;
import java.util.Iterator;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class Outake {
    public static void main(String[] args) {
        ArrayList<String> myList = new ArrayList<String>();
        myList.add(Arm);
        myList.add("Item 2");
        myList.add("Item 3");

        Iterator<String> iterator = myList.iterator();

        while (iterator.hasNext()) {
            String item = iterator.next();
            System.out.println(item);
        }
    }



}
