package org.firstinspires.ftc.teamcode.control;
public final class Angles {
 private Angles() {}
 public static double wrap(double a){return Math.atan2(Math.sin(a),Math.cos(a));}
 public static double clamp(double x,double lo,double hi){return Math.max(lo,Math.min(hi,x));}
 public static double deadband(double x){return Math.abs(x)<.05?0:x;}
}
