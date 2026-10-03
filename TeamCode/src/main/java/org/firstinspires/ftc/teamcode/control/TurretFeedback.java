package org.firstinspires.ftc.teamcode.control;
import org.firstinspires.ftc.teamcode.config.VisionConfig;
/** Circular encoder reduction; never differentiate across the voltage wrap directly. */
public final class TurretFeedback {
 public double angle=Double.NaN,velocity=0,disagreement=Double.NaN;
 public boolean healthy; private long previousTime;
 public void update(double lv,double rv,long now,VisionConfig c){
  healthy=Double.isFinite(lv)&&Double.isFinite(rv)&&lv>.001&&rv>.001&&lv<=c.analogRange+.05&&rv<=c.analogRange+.05;
  double a=Angles.wrap(c.leftSign*lv/c.analogRange*2*Math.PI-c.leftZero);
  double b=Angles.wrap(c.rightSign*rv/c.analogRange*2*Math.PI-c.rightZero);
  disagreement=Math.abs(Angles.wrap(a-b)); healthy &= c.turretCalibrated&&disagreement<=c.feedbackTolerance;
  double next=Angles.wrap(a+Angles.wrap(b-a)/2);
  velocity=healthy&&Double.isFinite(angle)&&now>previousTime&&now-previousTime<250000000L?
    Angles.wrap(next-angle)/((now-previousTime)*1e-9):0;
  angle=healthy?next:Double.NaN;previousTime=now;
 }
}
