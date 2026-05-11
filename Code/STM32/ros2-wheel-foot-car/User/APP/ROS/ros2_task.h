#ifndef __ROS2_TASK_H
#define __ROS2_TASK_H

#include "main.h"
#include "stdint.h"
#include "INS_task.h"
#include "chassisR_task.h"
#include "VMC_calc.h"
#include "bsp_dwt.h"
#include "usbd_cdc_if.h"

/* -------------------------------------------------------------------------- */
/* Timing                                                                     */
/* -------------------------------------------------------------------------- */
#define ROS2_TASK_PERIOD_MS   20U    /* 100Hz telemetry */
#define ROS2_CMD_TIMEOUT_MS   500U   /* command timeout → force idle */

/* -------------------------------------------------------------------------- */
/* Frame protocol constants                                                   */
/* -------------------------------------------------------------------------- */
#define ROS2_FRAME_HEADER     0xAA55u

#define ROS2_FRAME_TYPE_TLM   0x01u   /* STM32 -> ROS2 telemetry */
#define ROS2_FRAME_TYPE_CMD   0x02u   /* ROS2 -> STM32 command  */
#define ROS2_FRAME_TYPE_ACK   0x03u   /* STM32 -> ROS2 ack      */

#define ROS2_TLM_BUF_SIZE     256u
#define ROS2_CMD_BUF_SIZE     64u

/* -------------------------------------------------------------------------- */
/* Command flags (cmd_flags byte)                                             */
/* -------------------------------------------------------------------------- */
#define ROS2_CMD_ENABLE       0x01u   /* enable chassis control */
#define ROS2_CMD_JUMP         0x02u   /* trigger jump sequence */
#define ROS2_CMD_ESTOP        0x04u   /* emergency stop */
#define ROS2_CMD_RECOVER      0x08u   /* self-recovery from fall */

/* -------------------------------------------------------------------------- */
/* Telemetry packet  (STM32 -> ROS2, 100 Hz)                                  */
/* -------------------------------------------------------------------------- */
#pragma pack(push, 1)
typedef struct {
    float timestamp;         /* seconds from boot (DWT) */

    /* attitude */
    float roll, pitch, yaw;  /* rad */

    /* gyro (body frame) */
    float gyro_x, gyro_y, gyro_z;   /* rad/s */

    /* acceleration (world frame) */
    float accel_x, accel_y, accel_z; /* m/s^2 */

    /* odometry (world frame horizontal) */
    float vel_n;             /* m/s */
    float pos_n;             /* m */

    /* leg state - left */
    float theta_L;           /* rad, leg angle */
    float L0_L;              /* m, leg length */
    float wheel_T_L;         /* Nm, wheel torque */
    float Tp_L;              /* Nm, hip torque */
    float d_theta_L;         /* rad/s, leg angular velocity */

    /* leg state - right */
    float theta_R;
    float L0_R;
    float wheel_T_R;
    float Tp_R;
    float d_theta_R;

    /* power */
    float battery_voltage;   /* V */

    /* status flags */
    uint8_t start_flag;      /* 0=stopped, 1=running */
    uint8_t jump_flag;       /* jump state machine phase */
    uint8_t contact_L;       /* left ground contact */
    uint8_t contact_R;       /* right ground contact */
} ros2_telemetry_t;
#pragma pack(pop)

/* -------------------------------------------------------------------------- */
/* Command packet   (ROS2 -> STM32)                                           */
/* -------------------------------------------------------------------------- */
#pragma pack(push, 1)
typedef struct {
    float timestamp;         /* seconds (ROS2 wall time) */
    float v_set;             /* m/s target velocity */
    float yaw_rate_set;      /* rad/s target yaw rate */
    float roll_set;          /* rad target roll */
    float leg_set;           /* m target leg length */
    float pitch_set;         /* rad target pitch offset */
    uint8_t cmd_flags;       /* bit field, see ROS2_CMD_* */
} ros2_command_t;
#pragma pack(pop)

/* -------------------------------------------------------------------------- */
/* Authority flag — Xbox task checks this to yield control                    */
/* -------------------------------------------------------------------------- */
extern uint8_t ros2_active;   /* 0=Xbox may control, 1=ROS2 has authority */

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */
void ROS_Task(void const *argument);
void ROS2_Init(void);

#endif /* __ROS2_TASK_H */
