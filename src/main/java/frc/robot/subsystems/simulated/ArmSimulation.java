package frc.robot.subsystems.simulated;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.ArmSubsystem;

/**
 * A simulation of a "shoulder-joint" robotic arm, with associated rendering logic.
 * 
 * This takes control of a simulated potentiometer and reads a CANSparkMax
 * to drive an arm simulation.
 */
public class ArmSimulation {
    public static final double ARM_MASS_KG = 8.0;
    public static final double ARM_BASE_LENGTH_M = Units.inchesToMeters(28);
    public static final double ARM_GEAR_REDUCTION = 2.0;
    public static final double EXTENSION_MAX_SPEED_M_S = 0.25;
    public static final double GRIPPER_MAX_SPEED_M_S = 90.0;

    private AnalogPotentiometerSimulation m_armPotentiometer;
    private CANSparkMax m_armMotor;
    private AnalogPotentiometerSimulation m_extensionPotentiometer;
    private CANSparkMax m_extensionMotor;
    private AnalogPotentiometerSimulation m_gripperPotentiometer;
    private CANSparkMax m_gripperMotor;

    private DCMotor m_armGearbox;
    private BeattieSingleJointedArmSim m_armSim;
    private Mechanism2d m_armWindowDisplay;
    private MechanismRoot2d m_armRootDisplay;
    private MechanismLigament2d m_armTowerDisplay;
    private MechanismLigament2d m_armExtensionDisplay;
    private MechanismLigament2d m_armDisplay;
    private MechanismLigament2d m_gripperDisplay;

    public ArmSimulation(AnalogPotentiometerSimulation armPotentiometer,
    CANSparkMax armMotor,
    AnalogPotentiometerSimulation extensionPotentiometer,
    CANSparkMax extensionMotor,
    AnalogPotentiometerSimulation gripperPotentiometer,
    CANSparkMax gripperMotor) {
        m_armPotentiometer = armPotentiometer;
        m_armMotor = armMotor;
        m_extensionPotentiometer = extensionPotentiometer;
        m_extensionMotor = extensionMotor;
        m_gripperPotentiometer = gripperPotentiometer;
        m_gripperMotor = gripperMotor;

        m_armGearbox = DCMotor.getFalcon500(2);
        m_armSim = new BeattieSingleJointedArmSim(
            m_armGearbox, 
            ARM_GEAR_REDUCTION, 
            SingleJointedArmSim.estimateMOI(ARM_BASE_LENGTH_M, ARM_MASS_KG), 
            ARM_BASE_LENGTH_M, 
            Units.degreesToRadians(50),
            Units.degreesToRadians(360 - 50),
            true);

        m_armWindowDisplay = new Mechanism2d(140, 140);
        m_armRootDisplay = m_armWindowDisplay.getRoot("ArmRoot", 70, 70);
        m_armTowerDisplay = m_armRootDisplay.append(new MechanismLigament2d(
            "ArmTower", 
            30, -90));
        m_armDisplay = m_armRootDisplay.append(new MechanismLigament2d(
            "Arm",
            30, 
            Units.radiansToDegrees(m_armSim.getAngleRads()),
            6,
            new Color8Bit(Color.kYellow)));
        m_armExtensionDisplay = m_armDisplay.append(new MechanismLigament2d(
            "Arm Extension",
            0,
            0,
            6,
            new Color8Bit(Color.kAqua)));
        m_gripperDisplay = m_armExtensionDisplay.append(new MechanismLigament2d(
            "Gripper", 
            10, 0, 6, new Color8Bit(Color.kGreen)));

             m_extensionPotentiometer.set(0);
             m_gripperPotentiometer.set(180);

        SmartDashboard.putData("Arm Sim", m_armWindowDisplay);
    }

    /**
     * Call this every frame from simulationPeriodic to update the simulated arm.
     */
    public void calculate() {
        
        // Note: arm extension is in inches
        var currentArmExtensionMeters = Units.inchesToMeters(m_extensionPotentiometer.get());

        m_armSim.setArmLengthMeters(currentArmExtensionMeters + ARM_BASE_LENGTH_M);

        m_armSim.setInput(m_armMotor.get() * 12 /* volts */);
        m_armSim.update(0.02);

        var newAngleDegrees = Units.radiansToDegrees(m_armSim.getAngleRads());
        m_armPotentiometer.set(newAngleDegrees);
        m_armDisplay.setAngle(270 - newAngleDegrees);

        currentArmExtensionMeters += m_extensionMotor.get() * EXTENSION_MAX_SPEED_M_S;
        currentArmExtensionMeters = MathUtil.clamp(currentArmExtensionMeters, 0, Units.inchesToMeters(ArmSubsystem.MAX_ARM_EXTENSION_LENGTH_INCHES));
        m_extensionPotentiometer.set(Units.metersToInches(currentArmExtensionMeters));

        m_armExtensionDisplay.setLength(Units.metersToInches(currentArmExtensionMeters));

        var currentGripperAngle = m_gripperPotentiometer.get();

        currentGripperAngle += m_gripperMotor.get() * GRIPPER_MAX_SPEED_M_S;
        currentGripperAngle = MathUtil.clamp(currentGripperAngle, 180-55, 180+55);
        m_gripperPotentiometer.set(currentGripperAngle);

        m_gripperDisplay.setAngle(180 + currentGripperAngle);
    }


}
