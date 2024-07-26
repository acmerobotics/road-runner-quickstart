package org.firstinspires.ftc.teamcode.Outdated_CenterStage.Our;

public class OdoPos {

    /*

    public double OdoPosCalcX () {
        int prev_left_encoder_pos = 0;
        int prev_right_encoder_pos = 0;
        int prev_center_encoder_pos = 0;

        //initial pose
        for (int i = 0; i < 1; i += 1) {
            prev_left_encoder_pos = robotHardware.par0.getCurrentPosition();
            prev_right_encoder_pos = robotHardware.par1.getCurrentPosition();
            prev_center_encoder_pos = robotHardware.perp.getCurrentPosition();
        }

        //current pose
        double left_encoder_pos = robotHardware.par0.getCurrentPosition();
        double right_encoder_pos = robotHardware.par1.getCurrentPosition();
        double center_encoder_pos = robotHardware.perp.getCurrentPosition();

        //changing pose
        double delta_left_encoder_pos = left_encoder_pos - prev_left_encoder_pos;
        double delta_right_encoder_pos = right_encoder_pos - prev_right_encoder_pos;
        double delta_center_encoder_pos = center_encoder_pos - prev_center_encoder_pos;

        double delta_middle_pos = ((delta_left_encoder_pos + delta_right_encoder_pos) / 2);
        double delta_perp_pos = (delta_center_encoder_pos - (-1322.4924337170135 * TrackWidth));

        //exact change in pose
        double delta_x = (delta_middle_pos * Math.cos(rx) - delta_perp_pos * Math.sin(rx));
        double delta_y = (delta_middle_pos * Math.sin(rx) + delta_perp_pos * Math.cos(rx));

        double x_pos = Math.abs(delta_x);
        double y_pos = delta_y;

        return x_pos;
    }

     */
}
