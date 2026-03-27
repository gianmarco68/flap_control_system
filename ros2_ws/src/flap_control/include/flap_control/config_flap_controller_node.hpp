// ros2_ws/src/flap_control/include/flap_control/config_flap_controller_node.hpp
#pragma once
#include <cstdint> // Serve per uint8_t, uint16_t

namespace FlapConfig {

    // === PARAMETRI GEOMETRICI ===
    // Offset statico da aggiungere alla wand
    static constexpr double STATIC_OFFSET_DEG = 0.0;

    // === PARAMETRI CAN BUS ===
    // ID del messaggio CAN (PGN FF04 -> 65284)
    static constexpr int MOTOR_CAN_ID = 0xFF0400;

    // Manufacturer Code (Byte 0 e 1 del payload)
    // 1365 dec = 0x0555
    static constexpr uint8_t MANUF_CODE_MSB = 0x05;
    static constexpr uint8_t MANUF_CODE_LSB = 0x55; // Include Industry code?

    // === SICUREZZA ===
    static constexpr double MIN_ANGLE_DEG = 0.0;
    static constexpr double MAX_ANGLE_DEG = 180.0;

    // === DEBUG ===
    // Ogni quanti millisecondi stampare il log
    static constexpr int LOG_THROTTLE_MS = 1000; 

}