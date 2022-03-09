// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

  final WPI_TalonFX leftArmMotor = new WPI_TalonFX(CanBusId.LeftClimberMotor);
  final WPI_TalonFX rightArmMotor = new WPI_TalonFX(CanBusId.RightClimberMotor);

  final TalonFXSensorCollection leftArmSensorCollection = leftArmMotor.getSensorCollection();
  final TalonFXSensorCollection rightArmSensorCollection = rightArmMotor.getSensorCollection();


  final DoubleSolenoid leftArmSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticChannel.LeftArmForward, PneumaticChannel.LeftArmReverse);
  final DoubleSolenoid rightArmSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticChannel.RightArmForward, PneumaticChannel.RightArmReverse);

  final DoubleSolenoid leftLockSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticChannel.LeftClimbLockForward, PneumaticChannel.LeftClimbLockReverse);
  final DoubleSolenoid rightLockSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticChannel.RightClimbLockForward, PneumaticChannel.RightClimbLockReverse);

  private boolean isClimberEnabled = false;

  private TalonFXSimCollection leftArmSim;
  private TalonFXSimCollection rightArmSim;

  public ClimberSubsystem() {
    leftArmSolenoid.set(ClimberConfig.initialArmPosition);
    rightArmSolenoid.set(ClimberConfig.initialArmPosition);

    leftArmMotor.configFactoryDefault();
    rightArmMotor.configFactoryDefault();

    leftArmMotor.configForwardSoftLimitThreshold(ClimberConfig.armExtensionLimit, 30);
    leftArmMotor.configReverseSoftLimitThreshold(ClimberConfig.armRetractionLimit, 30);
    leftArmMotor.configForwardSoftLimitEnable (true, 30);
    leftArmMotor.configReverseSoftLimitEnable(true, 30);

    rightArmMotor.configForwardSoftLimitThreshold(ClimberConfig.armExtensionLimit, 30);
    rightArmMotor.configReverseSoftLimitThreshold(ClimberConfig.armRetractionLimit, 30);
    rightArmMotor.configForwardSoftLimitEnable (true, 30);
    rightArmMotor.configReverseSoftLimitEnable(true, 30);

    SmartDashboard.putNumber("Left Arm Length", leftArmSensorCollection.getIntegratedSensorPosition());
    SmartDashboard.putNumber("Right Arm Length", rightArmSensorCollection.getIntegratedSensorPosition());

    rightArmMotor.setInverted(TalonFXInvertType.Clockwise);

    if (RobotBase.isSimulation() == true) {
      leftArmSim = leftArmMotor.getSimCollection();
      rightArmSim = rightArmMotor.getSimCollection();
    }
  }

  @Override
  public void periodic() {

    if (leftArmMotor.isFwdLimitSwitchClosed() == 1 || leftArmMotor.isRevLimitSwitchClosed() == 1) {
      leftLockSolenoid.set(ClimberConfig.lockValue);
    }
    else if (rightArmMotor.isFwdLimitSwitchClosed() == 1 || rightArmMotor.isRevLimitSwitchClosed() == 1) {
      rightLockSolenoid.set(ClimberConfig.lockValue);
    }

    SmartDashboard.putBoolean("Climber Enabled", isClimberEnabled);
    SmartDashboard.putString("Left Arm Position", leftArmSolenoid.get() == ClimberConfig.upTilt ? "Up" : "Down");
    SmartDashboard.putString("Right Arm Position", rightArmSolenoid.get() == ClimberConfig.upTilt ? "Up" : "Down");
    SmartDashboard.putNumber("Left Arm Length", leftArmSensorCollection.getIntegratedSensorPosition());
    SmartDashboard.putNumber("Right Arm Length", -rightArmSensorCollection.getIntegratedSensorPosition());
    SmartDashboard.putString("Left Lock", leftLockSolenoid.get() == ClimberConfig.lockValue ? "Lock" : "Unlock");
    SmartDashboard.putString("Right Lock", rightLockSolenoid.get() == ClimberConfig.lockValue ? "Lock" : "Unlock");
  }

  public void simulationPeriodic() {

    int ticksPerPeriodic = 1;

    double currentLeftArmPosition = leftArmMotor.getSensorCollection().getIntegratedSensorPosition();
    int newLeftArmPosition = (int) (currentLeftArmPosition + leftArmSim.getMotorOutputLeadVoltage() * ticksPerPeriodic);

    double currentRightArmPosition = rightArmMotor.getSensorCollection().getIntegratedSensorPosition();
    int newRightArmPosition = (int) (currentRightArmPosition + rightArmSim.getMotorOutputLeadVoltage() * ticksPerPeriodic);

    leftArmSim.setIntegratedSensorRawPosition(newLeftArmPosition);
    rightArmSim.setIntegratedSensorRawPosition(newRightArmPosition);
  }

  public void enableClimber() {
    isClimberEnabled = true;
  }

  public void toggleArmPositions() {
    if (isClimberEnabled) {
      leftArmSolenoid.toggle();
      rightArmSolenoid.toggle();
    }
  }

  public void tiltArmsDown() {
    if (isClimberEnabled){
      leftArmSolenoid.set(ClimberConfig.downTilt);
      rightArmSolenoid.set(ClimberConfig.downTilt);
    }
  }

  public void tiltArmsUp() {
    if (isClimberEnabled){
      leftArmSolenoid.set(ClimberConfig.upTilt);
      rightArmSolenoid.set(ClimberConfig.upTilt);
    }
  }

  public void extendArms() {
    if (isClimberEnabled) {
      leftLockSolenoid.set(ClimberConfig.unlockValue);
      rightLockSolenoid.set(ClimberConfig.unlockValue);
      leftArmMotor.set(ClimberConfig.extendArmSpeed);
      rightArmMotor.set(ClimberConfig.extendArmSpeed);
    }
  }

  public void retractArms() {
    if (isClimberEnabled) {
      leftLockSolenoid.set(ClimberConfig.unlockValue);
      rightLockSolenoid.set(ClimberConfig.unlockValue);
      leftArmMotor.set(ClimberConfig.retractArmSpeed);
      rightArmMotor.set(ClimberConfig.retractArmSpeed);
    }
  }

  public void stopArms() {
    leftLockSolenoid.set(ClimberConfig.lockValue);
    rightLockSolenoid.set(ClimberConfig.lockValue);
    leftArmMotor.set(0);
    rightArmMotor.set(0);
  }

  public boolean isFullyExtended() {
    // TODO: replace with calls to motor
    return leftArmSensorCollection.getIntegratedSensorPosition() + 10 >= ClimberConfig.armExtensionLimit && rightArmSensorCollection.getIntegratedSensorPosition() + 10 >= ClimberConfig.armExtensionLimit;
    // return this.leftArmMotor.isFwdLimitSwitchClosed() == 1 && this.rightArmMotor.isFwdLimitSwitchClosed() == 1;
  }

  public boolean isFullyRetracted() {
    //TODO: replace with calls to motor
    return leftArmSensorCollection.getIntegratedSensorPosition() - 10 <= ClimberConfig.armRetractionLimit && rightArmSensorCollection.getIntegratedSensorPosition() - 10 <= ClimberConfig.armRetractionLimit;
    // return this.leftArmMotor.isRevLimitSwitchClosed() == 1 && this.rightArmMotor.isRevLimitSwitchClosed() == 1;
  }
}
