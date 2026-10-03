package org.firstinspires.ftc.teamcode.control;
public final class Readiness {
 private long since=-1;
 public boolean update(boolean good,long now,double seconds){
  if(!good){since=-1;return false;} if(since<0||now<since)since=now; return now-since>=seconds*1e9;
 }
 public static boolean wheels(double a,double b,double target,double tolerance){
  return Double.isFinite(target)&&target>0&&Double.isFinite(a)&&Double.isFinite(b)
    &&Math.abs(a-target)<=tolerance&&Math.abs(b-target)<=tolerance;
 }
}
