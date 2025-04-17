#pragma once

typedef struct position {
    double x; // Position X (mm)
    double y; // Position Y (mm)
    double theta; // Orientation (°)
} position_t;

typedef enum {
    /// @brief Success
    ret_OK = 0,

    /// @brief Fail
    ret_FAIL = -1,
} return_t;