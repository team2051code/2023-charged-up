package frc.robot.subsystems.simulated;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * A simulation of a "shoulder-joint" robotic arm, with associated rendering logic.
 * 
 * This takes control of a simulated potentiometer and reads a CANSparkMax
 * to drive an arm simulation.
 */
public class ArmSimulation {
    public static final double ARM_MASS_KG = 8.0;
    public static final double ARM_LENGTH_M = Units.inchesToMeters(30);
    public static final double ARM_GEAR_REDUCTION = 2.0;

    private AnalogPotentiometerSimulation m_simulatedPotentiometer;
    private CANSparkMax m_drivingMotor;

    private DCMotor m_armGearbox;
    private SingleJointedArmSim m_armSim;
    private Mechanism2d m_armWindowDisplay;
    private MechanismRoot2d m_armRootDisplay;
    private MechanismLigament2d m_armTowerDisplay;
    private MechanismLigament2d m_armDisplay;

    public ArmSimulation(AnalogPotentiometerSimulation simulatedPotentiometer,
    CANSparkMax drivingMotor) {
        m_simulatedPotentiometer = simulatedPotentiometer;
        m_drivingMotor = drivingMotor;

        m_armGearbox = DCMotor.getFalcon500(2);
        m_armSim = new SingleJointedArmSim(
            m_armGearbox, 
            ARM_GEAR_REDUCTION, 
            SingleJointedArmSim.estimateMOI(ARM_LENGTH_M, ARM_MASS_KG), 
            ARM_LENGTH_M, 
            Units.degreesToRadians(50),
            Units.degreesToRadians(360 - 50),
            true);

        m_armWindowDisplay = new Mechanism2d(60, 60);
        m_armRootDisplay = m_armWindowDisplay.getRoot("ArmRoot", 30, 30);
        m_armTowerDisplay = m_armRootDisplay.append(new MechanismLigament2d(
            "ArmTower", 
            30, -90));
        m_armDisplay = m_armRootDisplay.append(new MechanismLigament2d(
            "Arm",
            30, 
            Units.radiansToDegrees(m_armSim.getAngleRads()),
            6,
            new Color8Bit(Color.kYellow)));

        SmartDashboard.putData("Arm Sim", m_armWindowDisplay);
    }

    /**
     * Call this every frame from simulationPeriodic to update the simulated arm.
     */
    public void calculate() {
        SmartDashboard.putNumber("Simulated arm motor", m_drivingMotor.get());
        m_armSim.setInput(m_drivingMotor.get() * 12 /* volts */);
        m_armSim.update(0.02);
        SmartDashboard.putNumber("new arm angle", Units.radiansToDegrees(m_armSim.getAngleRads()));

        var newAngleDegrees = Units.radiansToDegrees(m_armSim.getAngleRads());
        m_simulatedPotentiometer.set(newAngleDegrees);
        m_armDisplay.setAngle(270 - newAngleDegrees);

    }


}
