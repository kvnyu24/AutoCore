#include "../src/core/bms/battery_manager.hpp"

int main() {
    autocore::bms::BatteryManager bms;
    
    // Configure thermal limits
    bms.setThermalLimits(0.0f, 45.0f);
    
    // Main loop
    while (true) {
        bms.updateBatteryState();
        
        if (bms.detectFaults()) {
            // Handle faults
            break;
        }
        
        // Balance cells if needed
        bms.balanceCells();
        
        // Get battery status
        float soc = bms.getStateOfCharge();
        float soh = bms.getStateOfHealth();
        
        // Use the battery state information
    }
    
    return 0;
} 