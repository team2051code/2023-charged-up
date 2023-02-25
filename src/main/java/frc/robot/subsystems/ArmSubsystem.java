package frc.robot.subsystems;

import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.LimitedMotor;

public class ArmSubsystem extends SubsystemBase {
    public final static double kArmP = 0; public final static double kArmI = 0; public final static double kArmD = 0;
    public final static double kextenderP = 0; public final static double kextenderI = 0; public final static double kextenderD = 0;
    public final static double kgripperP = 0; public final static double kgripperI = 0; public final static double kgripperD = 0;
    private final PIDController m_armPIDController;
    private final PIDController m_extenderPIDController;
    private final PIDController m_gripperPivotPIDController;
    private Solenoid m_gripperSolenoid;
    private Solenoid m_breakSolenoid;
    private RelativeEncoder m_armPivotEncoder;
    private RelativeEncoder m_extenderEncoder;
    private RelativeEncoder m_gripperRotatorEncoder;
    private RelativeEncoder m_gripperPivotEncoder;
    private RelativeEncoder m_intakeEncoder;
    private final CANSparkMax m_ArmPivot1;
    private final CANSparkMax m_ArmPivot2;
    private final CANSparkMax m_Extender;
    private final CANSparkMax m_GripperRotator;
    private final CANSparkMax m_GripperPivot;
    private final CANSparkMax m_IntakeLeft;
    private final CANSparkMax m_IntakeRight;
    private final MotorControllerGroup m_intake;
    private final MotorControllerGroup m_armPivot;
    public final static double POWER_LIMIT = 0;
    private AnalogPotentiometer m_absArmPivotEncoder;
    private AnalogPotentiometer m_absExtenderEncoder;
    private AnalogPotentiometer m_absGripperPivotEncoder;

    public ArmSubsystem(){
        m_ArmPivot1 = new LimitedMotor(CompetitionDriveConstants.kArmPivotMotorPort1, MotorType.kBrushless, POWER_LIMIT);
        m_ArmPivot2 = new LimitedMotor(CompetitionDriveConstants.kArmPivotMotorPort2, MotorType.kBrushless, POWER_LIMIT);
        m_Extender = new LimitedMotor(CompetitionDriveConstants.kExtenderMotorPort, MotorType.kBrushless, POWER_LIMIT);
        m_GripperRotator = new LimitedMotor(CompetitionDriveConstants.kGripperRotatorMotorPort, MotorType.kBrushless, POWER_LIMIT);
        m_GripperPivot = new LimitedMotor(CompetitionDriveConstants.kGripperPivotMotorPort, MotorType.kBrushless, POWER_LIMIT);
        m_IntakeLeft = new LimitedMotor(CompetitionDriveConstants.kIntakeLeft, MotorType.kBrushless, POWER_LIMIT);
        m_IntakeRight = new LimitedMotor(CompetitionDriveConstants.kIntakeRight, MotorType.kBrushless, POWER_LIMIT);
        m_intake = new MotorControllerGroup(m_IntakeLeft, m_IntakeRight);
        m_armPivot = new MotorControllerGroup(m_ArmPivot1, m_ArmPivot2);
        m_breakSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 1);
        m_gripperSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 2);
        m_armPivotEncoder = m_ArmPivot1.getEncoder();
        m_extenderEncoder = m_Extender.getEncoder();
        m_gripperRotatorEncoder = m_GripperRotator.getEncoder();
        m_gripperPivotEncoder = m_GripperPivot.getEncoder();
        m_intakeEncoder = m_IntakeRight.getEncoder();
        m_absArmPivotEncoder = new AnalogPotentiometer(0, 360, 0);
        m_absExtenderEncoder = new AnalogPotentiometer(1, 40, 0);
        m_absGripperPivotEncoder = new AnalogPotentiometer(2,360, 0);
        m_armPIDController = new PIDController(kArmP, kArmI, kArmD);
        m_extenderPIDController = new PIDController(kextenderP, kextenderI, kextenderD);
        m_gripperPivotPIDController = new PIDController(kgripperP, kgripperI, kgripperD);
        m_Extender.restoreFactoryDefaults();
        m_GripperPivot.restoreFactoryDefaults();
        m_GripperRotator.restoreFactoryDefaults();
        m_ArmPivot1.restoreFactoryDefaults();
        m_ArmPivot2.restoreFactoryDefaults();
        m_IntakeLeft.restoreFactoryDefaults();
        m_IntakeRight.restoreFactoryDefaults();
    }

    @Override
    public void periodic() {
        var armPivotVoltage = m_armPIDController.calculate(m_absArmPivotEncoder.get());
        var extenderVoltage = m_extenderPIDController.calculate(m_absExtenderEncoder.get());
        var gripperPivotVoltage = m_gripperPivotPIDController.calculate(m_absGripperPivotEncoder.get());
        if(m_absArmPivotEncoder.get()>45 && m_absArmPivotEncoder.get() < 325) //imagining 0 means vertically down
            m_armPivot.setVoltage(armPivotVoltage);
        if(!(m_absExtenderEncoder.get() == 40 && extenderVoltage>0)||!(m_absExtenderEncoder.get()==0 && extenderVoltage < 0))
            m_Extender.setVoltage(extenderVoltage);
        
        m_GripperPivot.setVoltage(gripperPivotVoltage);
    }

    public void incrementArmPivotSetpoint(double increment){
        double setpoint = m_armPIDController.getSetpoint();
        m_armPIDController.setSetpoint(setpoint+(increment/50.0));
    }


    public void setArmPivotSetpoint(double setpoint){
        m_armPIDController.setSetpoint(setpoint);
    }

    public void setExtenderSetpoint(double setpoint){
        m_extenderPIDController.setSetpoint(setpoint);
    }
    
    public void setGripperPivotSetpoint(double setpoint){
        m_gripperPivotPIDController.setSetpoint(setpoint);
    }

    public void resetExtenderEnc(){
        m_extenderEncoder.setPosition(0);
    }

    public void resetArmPivotEnc(){
        m_armPivotEncoder.setPosition(0);
    }

    public void resetGripperRotatorEnc(){
        m_armPivotEncoder.setPosition(0);
    }

    public void resetGripperPivotEnc(){
        m_gripperPivotEncoder.setPosition(0);
    }

    public void resetIntakeEnc(){
        m_intakeEncoder.setPosition(0);
    }

    public double getExtendorEnc(){
        return m_extenderEncoder.getPosition();
    }

    public double getArmPivotEnc(){
        return m_armPivotEncoder.getPosition();
    }

    public double getGripperRotatorEnc(){
        return m_gripperRotatorEncoder.getPosition();
    }

    public double getGripperPivotEnc(){
        return m_gripperPivotEncoder.getPosition();
    }

    public double getIntakeEnc(){
        return m_intakeEncoder.getPosition();
    }

    public void toggleBreak(){
        m_breakSolenoid.toggle();
    }

    public boolean getBreakSol(){
        return m_breakSolenoid.get();
    }
    
    public void toggleGripper(){
        m_gripperSolenoid.toggle();
    }

    public boolean getGripperSol(){
        return m_gripperSolenoid.get();
    }

}
