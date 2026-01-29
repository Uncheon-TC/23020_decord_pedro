package org.firstinspires.ftc.teamcode.sub_const;

public class shooter_const {

    public static double turret_offset = 1.0630; //inch

    public static double SCORE_HEIGHT = 26; //inch
    public static double SCORE_ANGLE = Math.toRadians(-30); //rad
    //public static double PASS_THROUGH_POINT_RADIUS = 5; //inch

    public static double HOOD_MIN_ANGLE = Math.toRadians(35);
    public static double HOOD_MAX_ANGLE = Math.toRadians(60);
    public static double HOOD_SERVO_MIN = servo_pos_const.servo_hood_min;
    public static double HOOD_SERVO_MAX = servo_pos_const.servo_hood_max;

    public static double FLYWHEEL_TPR = 103.8;
    public static double WHEEL_RADIUS = 1.89;

    //pid const
    public static double shooter_p = 0.025;
    public static double shooter_i = 0;
    public static double shooter_d = 0.00002;
    public static double shooter_f = 0;
    public static double SHOOTER_ANGLE_TPR = 537.7;

    public static double flywheel_p = 250;
    public static double flywheel_i = 5;
    public static double flywheel_d = 5;
    public static double flywheel_f = 0;

    public static double turret_offset_y(double heading) {
        return turret_offset * Math.sin(heading);
    }

    public static double turret_offset_x(double heading) {
        return turret_offset * Math.cos(heading);
    }

}
