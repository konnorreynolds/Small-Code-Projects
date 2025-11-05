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

#include "../include/units_core.h"
#include "../include/units_physics.h"
#include "../include/units_robotics.h"
#include "../include/units_utilities.h"

#include <iostream>
#include <iomanip>

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
    std::cout << "========================================\n";
    std::cout << "  Basic Battery Monitoring\n";
    std::cout << "========================================\n\n";

    BatteryConfig config;
    RobotBatteryMonitor monitor(config);

    std::cout << "Battery Configuration:\n";
    std::cout << "  Nominal Voltage: " << config.nominalVoltage << " V\n";
    std::cout << "  Voltage Range: " << config.minVoltage << " - "
              << config.maxVoltage << " V\n";
    std::cout << "  Capacity: " << config.capacityAh << " Ah\n\n";

    // Simulate some readings
    std::cout << "Initial Reading:\n";
    monitor.update(12.6, 5.0, 0.0);

    std::cout << "  Voltage: " << std::fixed << std::setprecision(2)
              << monitor.getVoltage() << " V\n";
    std::cout << "  Current: " << monitor.getCurrent() << " A\n";
    std::cout << "  Power: " << monitor.getPower() << " W\n";
    std::cout << "  State of Charge: " << std::setprecision(1)
              << monitor.getStateOfCharge() << " %\n";
    std::cout << "  Health: " << monitor.getHealthStatus() << "\n";
    std::cout << "  Est. Runtime: " << std::setprecision(1)
              << monitor.getEstimatedRuntimeMinutes() << " minutes\n\n";
}

void demonstrateDischargeCycle() {
    std::cout << "========================================\n";
    std::cout << "  Battery Discharge Simulation\n";
    std::cout << "========================================\n\n";

    RobotBatteryMonitor monitor;

    std::cout << "Simulating 30-minute robot operation with 10A draw...\n\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Time(min) | Voltage(V) | Current(A) | Power(W) | SOC(%) | Status\n";
    std::cout << "----------|------------|------------|----------|--------|-------------\n";

    double dt = 60.0;  // 1 minute time steps
    double voltage = 12.8;
    double current = 10.0;

    for (int minutes = 0; minutes <= 30; minutes += 5) {
        // Simulate voltage drop under load (simplified model)
        voltage = 12.8 - (minutes / 30.0) * 2.0;  // Drops to 10.8V

        monitor.update(voltage, current, dt);

        std::cout << std::setw(8) << minutes << "  | ";
        std::cout << std::setw(10) << monitor.getVoltage() << " | ";
        std::cout << std::setw(10) << monitor.getCurrent() << " | ";
        std::cout << std::setw(8) << monitor.getPower() << " | ";
        std::cout << std::setw(6) << std::setprecision(1)
                  << monitor.getStateOfCharge() << " | ";
        std::cout << monitor.getHealthStatus();

        if (monitor.hasCriticalBatteryWarning()) {
            std::cout << " ⚠️ CRITICAL";
        } else if (monitor.hasLowBatteryWarning()) {
            std::cout << " ⚠️ LOW";
        }

        std::cout << "\n";
    }

    std::cout << "\nTotal energy consumed: "
              << monitor.getTotalEnergyUsed() << " Wh\n\n";
}

void demonstratePowerBudget() {
    std::cout << "========================================\n";
    std::cout << "  Power Budget Analysis\n";
    std::cout << "========================================\n\n";

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

    std::cout << "Subsystem Power Analysis @ 12.0V:\n\n";
    std::cout << "Subsystem           | Current(A) | Power(W)\n";
    std::cout << "--------------------|------------|----------\n";

    for (const auto& sub : subsystems) {
        double power = batteryVoltage * sub.currentDraw;
        totalCurrent += sub.currentDraw;

        std::cout << std::left << std::setw(20) << sub.name << "| ";
        std::cout << std::right << std::setw(10) << sub.currentDraw << " | ";
        std::cout << std::setw(8) << power << "\n";
    }

    std::cout << "--------------------|------------|----------\n";
    std::cout << std::left << std::setw(20) << "TOTAL" << "| ";
    std::cout << std::right << std::setw(10) << totalCurrent << " | ";
    std::cout << std::setw(8) << (batteryVoltage * totalCurrent) << "\n\n";

    // Calculate battery life
    double batteryCapacityAh = 18.0;
    double runtimeHours = batteryCapacityAh / totalCurrent;
    double runtimeMinutes = runtimeHours * 60.0;

    std::cout << "Battery Life Estimate:\n";
    std::cout << "  Capacity: " << batteryCapacityAh << " Ah\n";
    std::cout << "  Total Draw: " << totalCurrent << " A\n";
    std::cout << "  Runtime: " << runtimeMinutes << " minutes\n\n";

    if (runtimeMinutes < 15) {
        std::cout << "⚠️  Warning: Runtime is very short!\n";
        std::cout << "    Consider reducing power consumption.\n\n";
    } else if (runtimeMinutes < 30) {
        std::cout << "⚠️  Caution: Runtime is limited.\n";
        std::cout << "    Monitor battery closely during matches.\n\n";
    } else {
        std::cout << "✓  Runtime is sufficient for typical operations.\n\n";
    }
}

void demonstrateBatteryWarnings() {
    std::cout << "========================================\n";
    std::cout << "  Battery Warning System\n";
    std::cout << "========================================\n\n";

    RobotBatteryMonitor monitor;

    std::cout << "Testing warning thresholds...\n\n";

    double testVoltages[] = {12.6, 12.0, 11.5, 11.0, 10.5, 10.0};

    for (double voltage : testVoltages) {
        monitor.update(voltage, 5.0, 0.1);

        std::cout << "Voltage: " << std::setprecision(2) << voltage << " V → ";
        std::cout << "SOC: " << std::setprecision(1)
                  << monitor.getStateOfCharge() << "% → ";

        if (monitor.hasCriticalBatteryWarning()) {
            std::cout << "🚨 CRITICAL - STOP ROBOT IMMEDIATELY!\n";
        } else if (monitor.hasLowBatteryWarning()) {
            std::cout << "⚠️  LOW BATTERY - Return to pit soon\n";
        } else {
            std::cout << "✓ Battery OK\n";
        }
    }

    std::cout << "\n";
}

// ============================================================================
// Main Program
// ============================================================================
int main() {
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════╗\n";
    std::cout << "║  Battery Management System             ║\n";
    std::cout << "║  Monitor, Track, and Protect           ║\n";
    std::cout << "╚════════════════════════════════════════╝\n";
    std::cout << "\n";

    demonstrateBasicMonitoring();
    demonstrateDischargeCycle();
    demonstratePowerBudget();
    demonstrateBatteryWarnings();

    std::cout << "========================================\n";
    std::cout << "  Key Takeaways\n";
    std::cout << "========================================\n\n";

    std::cout << "Battery Management Best Practices:\n\n";

    std::cout << "1. Always Monitor Voltage\n";
    std::cout << "   • Track voltage continuously\n";
    std::cout << "   • Use filtered values to reduce noise\n";
    std::cout << "   • Set appropriate warning thresholds\n\n";

    std::cout << "2. Track Power Consumption\n";
    std::cout << "   • Know your subsystem current draws\n";
    std::cout << "   • Calculate total power budget\n";
    std::cout << "   • Estimate runtime accurately\n\n";

    std::cout << "3. Implement Safety Warnings\n";
    std::cout << "   • Low battery warning (11.5V)\n";
    std::cout << "   • Critical warning (10.5V)\n";
    std::cout << "   • Auto-disable high-power systems if needed\n\n";

    std::cout << "4. Understand Battery Behavior\n";
    std::cout << "   • Voltage drops under load\n";
    std::cout << "   • Capacity decreases with age\n";
    std::cout << "   • Temperature affects performance\n\n";

    std::cout << "Real-World Applications:\n";
    std::cout << "  • FRC robot battery monitoring\n";
    std::cout << "  • Electric vehicle management\n";
    std::cout << "  • Drone flight controllers\n";
    std::cout << "  • Portable power systems\n\n";

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
