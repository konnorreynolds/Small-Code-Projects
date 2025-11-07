// ============================================================================
// Example 9: Battery Management System
// ============================================================================
// A practical example showing how to monitor and manage a robot's battery
// system with proper unit tracking and safety checks.
//
// Topics covered:
// - Voltage and current monitoring
// - State of charge calculation
// - Power consumption tracking
// - Battery health indicators
// - Low battery warnings
// - Electrical units (V, A, W, Ah)
// ============================================================================

#include "../../include/units_core.h"
#include "../../include/units_physics.h"
#include "../../include/units_robotics.h"
#include "../../include/units_utilities.h"


using namespace units;

// ============================================================================
// Battery Configuration
// ============================================================================
struct BatteryConfig {
    double nominalVoltage;      // 12V for most FRC batteries
    double minVoltage;          // Don't discharge below this
    double maxVoltage;          // Fully charged voltage
    double capacityAh;          // Amp-hours capacity

    BatteryConfig()
        : nominalVoltage(12.0),
          minVoltage(10.5),
          maxVoltage(13.0),
          capacityAh(18.0) {}  // Typical FRC battery
};

// ============================================================================
// Robot Battery Monitor
// ============================================================================
class RobotBatteryMonitor {
private:
    BatteryConfig config_;
    double currentVoltage_;
    double currentCurrent_;  // Amperes
    double totalEnergyUsed_; // Watt-hours

    // Filtering
    MovingAverageFilter<10> voltageFilter_;
    MovingAverageFilter<10> currentFilter_;

    // Alerts
    bool lowBatteryWarning_;
    bool criticalBatteryWarning_;

public:
    RobotBatteryMonitor(const BatteryConfig& config = BatteryConfig())
        : config_(config),
          currentVoltage_(config.nominalVoltage),
          currentCurrent_(0.0),
          totalEnergyUsed_(0.0),
          lowBatteryWarning_(false),
          criticalBatteryWarning_(false) {}

    // Update with new sensor readings
    void update(double voltage, double current, double dt) {
        // Filter readings to reduce noise
        currentVoltage_ = voltageFilter_.update(voltage);
        currentCurrent_ = currentFilter_.update(current);

        // Track energy consumption (Power = Voltage × Current)
        double power = currentVoltage_ * currentCurrent_;  // Watts
        totalEnergyUsed_ += (power * dt) / 3600.0;  // Convert to Wh

        // Check for warning conditions
        lowBatteryWarning_ = (currentVoltage_ < 11.5);
        criticalBatteryWarning_ = (currentVoltage_ < config_.minVoltage);
    }

    // Get current state of charge (0-100%)
    double getStateOfCharge() const {
        // Linear interpolation between min and max voltage
        double range = config_.maxVoltage - config_.minVoltage;
        double current = currentVoltage_ - config_.minVoltage;
        double soc = (current / range) * 100.0;
        return numerical::clamp(soc, 0.0, 100.0);
    }

    // Get estimated remaining capacity
    double getRemainingCapacityAh() const {
        return config_.capacityAh * (getStateOfCharge() / 100.0);
    }

    // Get estimated runtime at current draw
    double getEstimatedRuntimeMinutes() const {
        if (currentCurrent_ < 0.1) {
            return 999.0;  // Infinite (no draw)
        }
        double remainingAh = getRemainingCapacityAh();
        return (remainingAh / currentCurrent_) * 60.0;  // Minutes
    }

    // Getters
    double getVoltage() const { return currentVoltage_; }
    double getCurrent() const { return currentCurrent_; }
    double getPower() const { return currentVoltage_ * currentCurrent_; }
    double getTotalEnergyUsed() const { return totalEnergyUsed_; }

    bool hasLowBatteryWarning() const { return lowBatteryWarning_; }
    bool hasCriticalBatteryWarning() const { return criticalBatteryWarning_; }

    // Get battery health indicator
    std::string getHealthStatus() const {
        double soc = getStateOfCharge();
        if (soc > 75) return "Excellent";
        if (soc > 50) return "Good";
        if (soc > 25) return "Fair";
        if (soc > 10) return "Low";
        return "Critical";
    }
};

// ============================================================================
// Demonstration Scenarios
// ============================================================================

void demonstrateBasicMonitoring() {
    println("========================================");
    println("  Basic Battery Monitoring");
    println("========================================\n");

    BatteryConfig config;
    RobotBatteryMonitor monitor(config);

    println("Battery Configuration:");
    println("  Nominal Voltage: ", config.nominalVoltage, " V");
    print("  Voltage Range: " ,  config.minVoltage , " - ");
    println("  Capacity: ", config.capacityAh, " Ah\n");

    // Simulate some readings
    println("Initial Reading:");
    monitor.update(12.6, 5.0, 0.0);

    print("  Voltage: " ,   monitor.getVoltage() , " V\n");
    println("  Current: ", monitor.getCurrent(), " A");
    println("  Power: ", monitor.getPower(), " W");
    print("  State of Charge: " , monitor.getStateOfCharge() , " %\n");
    println("  Health: ", monitor.getHealthStatus(), "");
    print("  Est. Runtime: " , monitor.getEstimatedRuntimeMinutes() , " minutes\n\n");
}

void demonstrateDischargeCycle() {
    println("========================================");
    println("  Battery Discharge Simulation");
    println("========================================\n");

    RobotBatteryMonitor monitor;

    println("Simulating 30-minute robot operation with 10A draw...\n");
    print(, );
    println("Time(min) | Voltage(V) | Current(A) | Power(W) | SOC(%) | Status");
    println("----------|------------|------------|----------|--------|-------------");

    double dt = 60.0;  // 1 minute time steps
    double voltage = 12.8;
    double current = 10.0;

    for (int minutes = 0; minutes <= 30; minutes += 5) {
        // Simulate voltage drop under load (simplified model)
        voltage = 12.8 - (minutes / 30.0) * 2.0;  // Drops to 10.8V

        monitor.update(voltage, current, dt);

        print(, minutes, "  | ");
        print(, monitor.getVoltage(), " | ");
        print(, monitor.getCurrent(), " | ");
        print(, monitor.getPower(), " | ");
        print(,  monitor.getStateOfCharge() , " | ");
        print(monitor.getHealthStatus());

        if (monitor.hasCriticalBatteryWarning()) {
            print(" ⚠️ CRITICAL");
        } else if (monitor.hasLowBatteryWarning()) {
            print(" ⚠️ LOW");
        }

        println("");
    }

    print("\nTotal energy consumed: "
              ,  monitor.getTotalEnergyUsed() , " Wh\n\n");
}

void demonstratePowerBudget() {
    println("========================================");
    println("  Power Budget Analysis");
    println("========================================\n");

    // Robot subsystem power consumption
    struct Subsystem {
        std::string name;
        double currentDraw;  // Amperes
    };

    Subsystem subsystems[] = {
        {"Drive Motors (4x)", 20.0},
        {"Arm Motor", 5.0},
        {"Intake", 3.0},
        {"Shooter Flywheel", 8.0},
        {"Pneumatics", 2.0},
        {"Electronics", 3.0}
    };

    double batteryVoltage = 12.0;
    double totalCurrent = 0.0;

    println("Subsystem Power Analysis @ 12.0V:\n");
    println("Subsystem           | Current(A) | Power(W)");
    println("--------------------|------------|----------");

    for (const auto& sub : subsystems) {
        double power = batteryVoltage * sub.currentDraw;
        totalCurrent += sub.currentDraw;

        print(, sub.name, "| ");
        print(std::right, sub.currentDraw, " | ");
        println(, power, "");
    }

    println("--------------------|------------|----------");
    print(, "TOTAL", "| ");
    print(std::right, totalCurrent, " | ");
    println(, (batteryVoltage * totalCurrent), "\n");

    // Calculate battery life
    double batteryCapacityAh = 18.0;
    double runtimeHours = batteryCapacityAh / totalCurrent;
    double runtimeMinutes = runtimeHours * 60.0;

    println("Battery Life Estimate:");
    println("  Capacity: ", batteryCapacityAh, " Ah");
    println("  Total Draw: ", totalCurrent, " A");
    println("  Runtime: ", runtimeMinutes, " minutes\n");

    if (runtimeMinutes < 15) {
        println("⚠️  Warning: Runtime is very short!");
        println("    Consider reducing power consumption.\n");
    } else if (runtimeMinutes < 30) {
        println("⚠️  Caution: Runtime is limited.");
        println("    Monitor battery closely during matches.\n");
    } else {
        println("✓  Runtime is sufficient for typical operations.\n");
    }
}

void demonstrateBatteryWarnings() {
    println("========================================");
    println("  Battery Warning System");
    println("========================================\n");

    RobotBatteryMonitor monitor;

    println("Testing warning thresholds...\n");

    double testVoltages[] = {12.6, 12.0, 11.5, 11.0, 10.5, 10.0};

    for (double voltage : testVoltages) {
        monitor.update(voltage, 5.0, 0.1);

        print("Voltage: ", voltage, " V → ");
        print("SOC: " , monitor.getStateOfCharge() , "% → ");

        if (monitor.hasCriticalBatteryWarning()) {
            println("🚨 CRITICAL - STOP ROBOT IMMEDIATELY!");
        } else if (monitor.hasLowBatteryWarning()) {
            println("⚠️  LOW BATTERY - Return to pit soon");
        } else {
            println("✓ Battery OK");
        }
    }

    println("");
}

// ============================================================================
// Main Program
// ============================================================================
int main() {
    println("");
    println("╔════════════════════════════════════════╗");
    println("║  Battery Management System             ║");
    println("║  Monitor, Track, and Protect           ║");
    println("╚════════════════════════════════════════╝");
    println("");

    demonstrateBasicMonitoring();
    demonstrateDischargeCycle();
    demonstratePowerBudget();
    demonstrateBatteryWarnings();

    println("========================================");
    println("  Key Takeaways");
    println("========================================\n");

    println("Battery Management Best Practices:\n");

    println("1. Always Monitor Voltage");
    println("   • Track voltage continuously");
    println("   • Use filtered values to reduce noise");
    println("   • Set appropriate warning thresholds\n");

    println("2. Track Power Consumption");
    println("   • Know your subsystem current draws");
    println("   • Calculate total power budget");
    println("   • Estimate runtime accurately\n");

    println("3. Implement Safety Warnings");
    println("   • Low battery warning (11.5V)");
    println("   • Critical warning (10.5V)");
    println("   • Auto-disable high-power systems if needed\n");

    println("4. Understand Battery Behavior");
    println("   • Voltage drops under load");
    println("   • Capacity decreases with age");
    println("   • Temperature affects performance\n");

    println("Real-World Applications:");
    println("  • FRC robot battery monitoring");
    println("  • Electric vehicle management");
    println("  • Drone flight controllers");
    println("  • Portable power systems\n");

    return 0;
}

/*
Expected Output:
    - Basic battery monitoring demonstration
    - Discharge cycle simulation over 30 minutes
    - Power budget analysis for robot subsystems
    - Warning system testing

This example demonstrates:
    - Practical electrical unit usage (V, A, W, Ah)
    - State of charge calculation
    - Power consumption tracking
    - Filtering sensor readings
    - Safety warning implementation
    - Battery health assessment
    - Runtime estimation

Perfect for:
    - FRC/VEX teams managing robot batteries
    - Understanding power systems
    - Implementing battery safety features
    - Learning electrical units in context
*/
