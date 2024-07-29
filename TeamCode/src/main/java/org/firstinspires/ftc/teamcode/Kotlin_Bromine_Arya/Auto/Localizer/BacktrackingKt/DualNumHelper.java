package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.Localizer.BacktrackingKt;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;

public class DualNumHelper {
    public static DualNum<Time> createDualNum(double [] list) {
            return new DualNum<>(list);
    }
}
