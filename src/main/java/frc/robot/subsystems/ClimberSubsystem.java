// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.roborio.CanBusId;
import frc.robot.config.roborio.PneumaticChannel;
import frc.robot.config.subsystems.ClimberConfig;

public class ClimberSubsystem extends SubsystemBase {

  //ticks per rev = 2048
  final WPI_TalonFX leftArmMotor = new WPI_TalonFX(CanBusId.LeftClimberMotor);
  final WPI_TalonFX rightArmMotor = new WPI_TalonFX(CanBusId.RightClimberMotor);

  final TalonFXSensorCollection leftArmSensorCollection = leftArmMotor.getSensorCollection();
  final TalonFXSensorCollection rightArmSensorCollection = rightArmMotor.getSensorCollection();

  final DoubleSolenoid leftArmSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticChannel.LeftArmForward, PneumaticChannel.LeftArmReverse);
  final DoubleSolenoid rightArmSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticChannel.RightArmForward, PneumaticChannel.RightArmReverse);

  final DoubleSolenoid lockSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticChannel.ClimbLockForward, PneumaticChannel.ClimbLockReverse);

  final DoubleSolenoid secondaryHookSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticChannel.SecondaryHookForward, PneumaticChannel.SecondaryHookReverse);

  private boolean isClimberEnabled = false;

  private TalonFXSimCollection leftArmSim;
  private TalonFXSimCollection rightArmSim;

  public ClimberSubsystem() {

    leftArmSolenoid.set(ClimberConfig.upTilt);
    rightArmSolenoid.set(ClimberConfig.upTilt);

    secondaryHookSolenoid.set(ClimberConfig.upTilt);

    lockSolenoid.set(ClimberConfig.unlockValue);

    leftArmMotor.configFactoryDefault();
    rightArmMotor.configFactoryDefault();

    setArmLimits(ClimberConfig.upTiltArmExtensionLimit, ClimberConfig.upTiltArmRetractionLimit);

    leftArmMotor.setNeutralMode(NeutralMode.Brake);
    rightArmMotor.setNeutralMode(NeutralMode.Brake);

    SmartDashboard.putNumber("Left Arm Length", leftArmSensorCollection.getIntegratedSensorPosition());
    SmartDashboard.putNumber("Right Arm Length", rightArmSensorCollection.getIntegratedSensorPosition());

    leftArmMotor.setInverted(TalonFXInvertType.Clockwise);

    if (RobotBase.isSimulation() == true) {
      leftArmSim = leftArmMotor.getSimCollection();
      rightArmSim = rightArmMotor.getSimCollection();
    }
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("Climber Enabled", isClimberEnabled);
    SmartDashboard.putString("Left Arm Position", leftArmSolenoid.get() == ClimberConfig.upTilt ? "Up" : "Down");
    SmartDashboard.putString("Right Arm Position", rightArmSolenoid.get() == ClimberConfig.upTilt ? "Up" : "Down");
    SmartDashboard.putString("Secondary Hook Position", secondaryHookSolenoid.get() == ClimberConfig.upTilt ? "Up" : "Down");
    SmartDashboard.putNumber("Left Arm Length", leftArmSensorCollection.getIntegratedSensorPosition());
    SmartDashboard.putNumber("Right Arm Length", -rightArmSensorCollection.getIntegratedSensorPosition());
    SmartDashboard.putBoolean("Climber Lock", isClimberLocked());
  }

  public void simulationPeriodic() {

    int ticksPerPeriodic = 100;

    double currentLeftArmPosition = leftArmMotor.getSensorCollection().getIntegratedSensorPosition();
    int newLeftArmPosition = (int) (currentLeftArmPosition + leftArmSim.getMotorOutputLeadVoltage() * ticksPerPeriodic);

    double currentRightArmPosition = rightArmMotor.getSensorCollection().getIntegratedSensorPosition();
    int newRightArmPosition = (int) (currentRightArmPosition + rightArmSim.getMotorOutputLeadVoltage() * ticksPerPeriodic);

    leftArmSim.setIntegratedSensorRawPosition(newLeftArmPosition);
    rightArmSim.setIntegratedSensorRawPosition(newRightArmPosition);
  }

  public void enableClimber() {
    isClimberEnabled = true;

    lockSolenoid.set(ClimberConfig.unlockValue);
  }

  public void disableClimber() {
    tiltArmsDown();
    tiltHooksDown();
    isClimberEnabled = false;
  }

  public void toggleArmPositions() {
    if (isClimberEnabled == false) return;

    if (leftArmSolenoid.get() == ClimberConfig.downTilt) {
      tiltArmsUp();
    }
    else {
      tiltArmsDown();
    }
  }

  public void tiltArmsDown() {

    leftArmSolenoid.set(ClimberConfig.downTilt);
    rightArmSolenoid.set(ClimberConfig.downTilt);

    setArmLimits(ClimberConfig.downTiltArmExtensionLimit, ClimberConfig.downTiltArmRetractionLimit);
  }

  public void tiltArmsUp() {
    if (isClimberEnabled == false) return;

    leftArmSolenoid.set(ClimberConfig.upTilt);
    rightArmSolenoid.set(ClimberConfig.upTilt);

    setArmLimits(ClimberConfig.upTiltArmExtensionLimit, ClimberConfig.upTiltArmRetractionLimit);
  }

  public void extendArms() {
    if (isClimberEnabled == false) return;

    leftArmMotor.set(ClimberConfig.extendArmSpeed);
    rightArmMotor.set(ClimberConfig.extendArmSpeed);
  }

  public void retractArms() {
    if (isClimberEnabled == false) return;

    leftArmMotor.set(ClimberConfig.retractArmSpeed);
    rightArmMotor.set(ClimberConfig.retractArmSpeed);
  }

  public void moveArms(double speed) {
    if (isClimberEnabled == false) return;

    double squaredSpeed = speed * Math.abs(speed);

    if (squaredSpeed < 0) {
     squaredSpeed =  squaredSpeed * 0.70;
    }

    leftArmMotor.set(squaredSpeed);
    rightArmMotor.set(squaredSpeed);
  }

  public void stopArms() {
    leftArmMotor.set(0);
    rightArmMotor.set(0);
  }

  public void toggleHookPositions() {
    if (isClimberEnabled == false) return;

    if (secondaryHookSolenoid.get() == ClimberConfig.downTilt) {
      tiltHooksUp();
    }
    else {
      tiltHooksDown();
    }
  }

  public void tiltHooksDown() {

    secondaryHookSolenoid.set(ClimberConfig.downTilt);

  }

  public void tiltHooksUp() {
    if (isClimberEnabled == false) return;

    secondaryHookSolenoid.set(ClimberConfig.upTilt);

  }

  public void toggleArmLocks() {
    if(isClimberEnabled == false) return;

    lockSolenoid.toggle();
  }

  public boolean isClimberLocked(){
    return lockSolenoid.get() == ClimberConfig.lockValue;
  }

  public boolean isEnabled() {
    return isClimberEnabled;
  }

  private void setArmLimits(int extensionLimit, int retractionLimit) {

    leftArmMotor.configForwardSoftLimitThreshold(extensionLimit, 30);
    leftArmMotor.configReverseSoftLimitThreshold(retractionLimit, 30);

    rightArmMotor.configForwardSoftLimitThreshold(extensionLimit, 30);
    rightArmMotor.configReverseSoftLimitThreshold(retractionLimit, 30);

    leftArmMotor.configForwardSoftLimitEnable (true, 30);
    leftArmMotor.configReverseSoftLimitEnable(true, 30);

    rightArmMotor.configForwardSoftLimitEnable (true, 30);
    rightArmMotor.configReverseSoftLimitEnable(true, 30);
  }
}
