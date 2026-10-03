package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.*;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import org.firstinspires.ftc.teamcode.config.*;
import org.firstinspires.ftc.teamcode.control.*;
public final class Turret extends SubsystemBase {
 private final ServoEx left,right;private final AbsoluteAnalogEncoder le,re;
 public final TurretFeedback feedback=new TurretFeedback();public final VisionConfig config;
 public double leftVoltage,rightVoltage;private double position=.5,targetAngle,lastCommandAngle;
 public Turret(HardwareMap hw,VisionConfig c){
  config=c;left=new ServoEx(hw,"turret1");right=new ServoEx(hw,"turret2");right.setInverted(true);
  left.setPwm(new PwmControl.PwmRange(500,2500));right.setPwm(new PwmControl.PwmRange(500,2500));
  le=new AbsoluteAnalogEncoder(hw,c.leftFeedback);re=new AbsoluteAnalogEncoder(hw,c.rightFeedback);position=c.servoCenter;
 }
 public void read(long now){leftVoltage=le.getVoltage();rightVoltage=re.getVoltage();feedback.update(leftVoltage,rightVoltage,now,config);}
 public double aim(double angle,double rate,double dt){
  if(!feedback.healthy||!Double.isFinite(angle)||!Double.isFinite(rate))return Double.NaN;
  double lo=(config.servoMin-config.servoCenter)*config.radiansPerServo,hi=(config.servoMax-config.servoCenter)*config.radiansPerServo;
  double wanted=Angles.wrap(angle),continuous=lastCommandAngle+Angles.wrap(wanted-lastCommandAngle);
  targetAngle=Angles.clamp(continuous,lo,hi);
  double led=Angles.clamp(targetAngle+rate*config.turretLag,lo,hi);
  lastCommandAngle+=Angles.clamp(led-lastCommandAngle,-config.maxTurretRate*dt,config.maxTurretRate*dt);
  position=config.servoCenter+lastCommandAngle/config.radiansPerServo;
  return Angles.wrap(wanted-targetAngle);
 }
 public boolean ready(){return feedback.healthy&&Math.abs(Angles.wrap(targetAngle-feedback.angle))<MechanismConfig.turretTolerance;}
 public void write(){if(config.turretCalibrated&&feedback.healthy){left.set(position);right.set(position);}}
 public void setRawPosition(double p){if(Double.isFinite(p)){position=Angles.clamp(p,0,1);left.set(position);right.set(position);}}
 public double getAngle(){return Math.toDegrees(feedback.angle);}public double getPosition(){return position;}
 public void hold(){if(feedback.healthy){lastCommandAngle=feedback.angle;targetAngle=feedback.angle;position=config.servoCenter+feedback.angle/config.radiansPerServo;write();}}
}
