#include "ros2_task.h"

#include <string.h>
#include "cmsis_os.h"
#include "usbd_cdc_if.h"
#include "power_task.h"

/* -------------------------------------------------------------------------- */
/* External data referenced from other tasks                                  */
/* -------------------------------------------------------------------------- */
extern INS_t        INS;
extern chassis_t    chassis_move;
extern vmc_leg_t    left;
extern vmc_leg_t    right;
extern float        jump_time;

/* CDC receive buffers — defined in usbd_cdc_if.c */
extern uint8_t  UserRxBufferHS[];

/* CDC notification flags — set in usbd_cdc_if.c USER CODE */
extern volatile uint32_t ros2_cdc_rx_len;
extern volatile uint8_t  ros2_cdc_rx_ready;

/* -------------------------------------------------------------------------- */
/* Private variables                                                          */
/* -------------------------------------------------------------------------- */
static ros2_telemetry_t telemetry;
static ros2_command_t   latest_command;

/* last time a valid command was received (HAL tick ms) */
static uint32_t last_cmd_tick;
uint8_t  ros2_active;   /* set after first ENABLE command received */

/* -------------------------------------------------------------------------- */
/* RX frame assembly state machine                                            */
/* -------------------------------------------------------------------------- */
typedef enum {
    RX_ST_WAIT_H0 = 0,
    RX_ST_WAIT_H1,
    RX_ST_WAIT_TYPE,
    RX_ST_WAIT_LEN,
    RX_ST_WAIT_DATA,
} ros2_rx_state_t;

static ros2_rx_state_t rx_state = RX_ST_WAIT_H0;
static uint8_t  rx_buf[ROS2_CMD_BUF_SIZE];  /* raw frame bytes */
static uint8_t  rx_payload_len;
static uint8_t  rx_idx;
static uint8_t  rx_type;
static uint8_t  cmd_frame_ready;            /* set when a complete valid frame is parsed */

/* -------------------------------------------------------------------------- */
/* CRC16-CCITT (polynomial 0x1021, initial 0xFFFF)                            */
/* -------------------------------------------------------------------------- */
static uint16_t CRC16_CCITT(const uint8_t *data, uint8_t len)
{
    uint16_t crc = 0xFFFFu;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000u) {
                crc = (crc << 1) ^ 0x1021u;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/* -------------------------------------------------------------------------- */
/* Feed one byte into the RX frame parser                                     */
/* -------------------------------------------------------------------------- */
static void ROS2_ParseRxByte(uint8_t byte)
{
    switch (rx_state) {
    case RX_ST_WAIT_H0:
        if (byte == 0xAAu) {
            rx_buf[0] = byte;
            rx_state = RX_ST_WAIT_H1;
        }
        break;

    case RX_ST_WAIT_H1:
        if (byte == 0x55u) {
            rx_buf[1] = byte;
            rx_state = RX_ST_WAIT_TYPE;
            rx_idx = 2;
        } else {
            rx_state = (byte == 0xAAu) ? RX_ST_WAIT_H1 : RX_ST_WAIT_H0;
        }
        break;

    case RX_ST_WAIT_TYPE:
        rx_buf[rx_idx++] = byte;
        rx_type = byte;
        rx_state = RX_ST_WAIT_LEN;
        break;

    case RX_ST_WAIT_LEN:
        rx_buf[rx_idx++] = byte;
        rx_payload_len = byte;
        if (rx_payload_len == 0) {
            /* no payload, jump to CRC directly */
            rx_state = RX_ST_WAIT_DATA;
        } else {
            rx_state = RX_ST_WAIT_DATA;
        }
        break;

    case RX_ST_WAIT_DATA:
        rx_buf[rx_idx++] = byte;
        /* total frame = header(2) + type(1) + len(1) + payload(N) + crc(2) */
        if (rx_idx >= (uint8_t)(4u + rx_payload_len + 2u)) {
            /* full frame received, verify CRC */
            uint8_t  hdr_len = 4u; /* header + type + len */
            uint16_t crc_calc = CRC16_CCITT(&rx_buf[2], rx_payload_len + 2u); /* over type+len+payload */
            uint16_t crc_recv = ((uint16_t)rx_buf[hdr_len + rx_payload_len])
                              | ((uint16_t)rx_buf[hdr_len + rx_payload_len + 1u] << 8);

            if (crc_calc == crc_recv && rx_type == ROS2_FRAME_TYPE_CMD) {
                cmd_frame_ready = 1;
            }
            rx_state = RX_ST_WAIT_H0;
        }
        break;

    default:
        rx_state = RX_ST_WAIT_H0;
        break;
    }
}

/* -------------------------------------------------------------------------- */
/* Unpack a raw command frame payload into ros2_command_t                     */
/* -------------------------------------------------------------------------- */
static void ROS2_UnpackCommand(const uint8_t *payload, uint8_t len,
                               ros2_command_t *cmd)
{
    if (len < sizeof(ros2_command_t) || payload == NULL || cmd == NULL) {
        return;
    }
    memcpy(cmd, payload, sizeof(ros2_command_t));
}

/* -------------------------------------------------------------------------- */
/* Collect telemetry data from global state                                   */
/* -------------------------------------------------------------------------- */
static void ROS2_CollectTelemetry(ros2_telemetry_t *tlm)
{
    tlm->timestamp = DWT_GetTimeline_s();

    /* attitude */
    tlm->roll  = INS.Roll;
    tlm->pitch = INS.Pitch;
    tlm->yaw   = INS.Yaw;

    /* gyro (body frame) */
    tlm->gyro_x = INS.Gyro[0];
    tlm->gyro_y = INS.Gyro[1];
    tlm->gyro_z = INS.Gyro[2];

    /* accel (world frame: MotionAccel_n) */
    tlm->accel_x = INS.MotionAccel_n[0];
    tlm->accel_y = INS.MotionAccel_n[1];
    tlm->accel_z = INS.MotionAccel_n[2];

    /* odometry */
    tlm->vel_n = INS.v_n;
    tlm->pos_n = INS.x_n;

    /* left leg */
    tlm->theta_L   = left.theta;
    tlm->L0_L      = left.L0;
    tlm->wheel_T_L = chassis_move.wheel_motor[1].wheel_T;
    tlm->Tp_L      = left.Tp;
    tlm->d_theta_L = left.d_theta;

    /* right leg */
    tlm->theta_R   = right.theta;
    tlm->L0_R      = right.L0;
    tlm->wheel_T_R = chassis_move.wheel_motor[0].wheel_T;
    tlm->Tp_R      = right.Tp;
    tlm->d_theta_R = right.d_theta;

    /* power */
    tlm->battery_voltage = GetBatteryVoltage();

    /* flags */
    tlm->start_flag = chassis_move.start_flag;
    tlm->jump_flag  = chassis_move.jump_flag;

    /* ground contact — defined in VMC_calc.c as global or from vmc_leg_t */
    extern uint8_t right_flag;
    extern uint8_t left_flag;
    tlm->contact_R = right_flag;
    tlm->contact_L = left_flag;
}

/* -------------------------------------------------------------------------- */
/* Pack telemetry into a frame buffer, return total frame length              */
/* -------------------------------------------------------------------------- */
static uint16_t ROS2_PackTelemetry(const ros2_telemetry_t *tlm,
                                   uint8_t *frame_buf)
{
    uint16_t payload_len = sizeof(ros2_telemetry_t);
    uint8_t  hdr_len = 4u;  /* header(2) + type(1) + len(1) */

    /* header */
    frame_buf[0] = 0xAAu;
    frame_buf[1] = 0x55u;
    frame_buf[2] = ROS2_FRAME_TYPE_TLM;
    frame_buf[3] = (uint8_t)payload_len;

    /* payload */
    memcpy(&frame_buf[hdr_len], tlm, payload_len);

    /* CRC over type + len + payload */
    uint16_t crc = CRC16_CCITT(&frame_buf[2], payload_len + 2u);
    frame_buf[hdr_len + payload_len]     = (uint8_t)(crc & 0xFFu);
    frame_buf[hdr_len + payload_len + 1u] = (uint8_t)((crc >> 8) & 0xFFu);

    return (uint16_t)(hdr_len + payload_len + 2u);
}

/* -------------------------------------------------------------------------- */
/* Apply received command to chassis state                                    */
/* -------------------------------------------------------------------------- */
static void ROS2_ApplyCommand(const ros2_command_t *cmd)
{
    if (cmd == NULL) return;

    /* emergency stop — override everything */
    if (cmd->cmd_flags & ROS2_CMD_ESTOP) {
        chassis_move.start_flag = 0;
        chassis_move.v_set  = 0.0f;
        // chassis_move.x_set  = chassis_move.x_filter;
        // chassis_move.leg_set = 0.08f;
        chassis_move.roll_set = 0.0f;
        return;
    }

    /* self-recovery */
    if (cmd->cmd_flags & ROS2_CMD_RECOVER) {
        chassis_move.recover_flag = 1;
        chassis_move.leg_set = 0.08f;
        return;
    }

    /* enable/disable chassis */
    if (cmd->cmd_flags & ROS2_CMD_ENABLE) {
        ros2_active = 1;  /* ROS2 has taken over */
        if (chassis_move.start_flag == 0) {
            chassis_move.start_flag = 1;
            chassis_move.turn_set = chassis_move.total_yaw;  /* align yaw on start */
        }
        /* apply control values */
        chassis_move.v_set    = cmd->v_set;
        chassis_move.roll_set = cmd->roll_set;
        chassis_move.leg_set  = cmd->leg_set;

        /* integrate yaw rate -> absolute yaw target */
        chassis_move.turn_set = chassis_move.turn_set
                              + cmd->yaw_rate_set * (ROS2_TASK_PERIOD_MS * 0.001f);

        /* integrate velocity -> position target */
        chassis_move.x_set = chassis_move.x_set
                           + cmd->v_set * (ROS2_TASK_PERIOD_MS * 0.001f);

        /* optional pitch offset */
        /* (LQR uses chassis->myPithL which is derived from INS.Pitch;
            pitch_set could be added as an offset in the control loop) */

        /* jump trigger — rising edge handled in chassis control */
        if (cmd->cmd_flags & ROS2_CMD_JUMP) {
            if (chassis_move.jump_flag == 0 && chassis_move.jump_flag2 == 0) {
                chassis_move.jump_flag  = 1;
                chassis_move.jump_flag2 = 1;
            }
        }
    } else {
        /* disable — release authority back to Xbox */
        chassis_move.start_flag = 0;
        chassis_move.v_set  = 0.0f;
        //chassis_move.x_set  = chassis_move.x_filter;
        chassis_move.leg_set = 0.08f;
        chassis_move.roll_set = 0.0f;
        ros2_active = 0;  /* allow Xbox to take over */
    }
}

/* -------------------------------------------------------------------------- */
/* Send telemetry frame over USB CDC                                          */
/* -------------------------------------------------------------------------- */
static void ROS2_SendTelemetry(void)
{
    static uint8_t frame_buf[ROS2_TLM_BUF_SIZE];

    ROS2_CollectTelemetry(&telemetry);
    uint16_t len = ROS2_PackTelemetry(&telemetry, frame_buf);

    CDC_Transmit_HS(frame_buf, len);
}

/* -------------------------------------------------------------------------- */
/* Process CDC received data (feed into frame parser)                         */
/* -------------------------------------------------------------------------- */
static void ROS2_ProcessCdcRx(void)
{
    if (ros2_cdc_rx_ready == 0) {
        return;
    }

    uint32_t len = ros2_cdc_rx_len;
    ros2_cdc_rx_ready = 0;

    if (len == 0 || len > APP_RX_DATA_SIZE) {
        return;
    }

    for (uint32_t i = 0; i < len; i++) {
        ROS2_ParseRxByte(UserRxBufferHS[i]);
    }
}

/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */
void ROS2_Init(void)
{
    memset(&telemetry,      0, sizeof(telemetry));
    memset(&latest_command, 0, sizeof(latest_command));
    memset(rx_buf,          0, sizeof(rx_buf));

    rx_state         = RX_ST_WAIT_H0;
    rx_idx           = 0;
    rx_payload_len   = 0;
    rx_type          = 0;
    cmd_frame_ready  = 0;
    ros2_active      = 0;
    last_cmd_tick    = HAL_GetTick();
}

/* -------------------------------------------------------------------------- */
/* FreeRTOS task entry                                                        */
/* -------------------------------------------------------------------------- */
void ROS_Task(void const *argument)
{
    (void)argument;

    osDelay(100);           /* wait for system init */
    ROS2_Init();

    for (;;) {
        /* --- process incoming CDC data --- */
        ROS2_ProcessCdcRx();

        /* --- if a complete valid command frame arrived, unpack & apply --- */
        if (cmd_frame_ready) {
            cmd_frame_ready = 0;
            uint8_t hdr_len = 4u;
            ROS2_UnpackCommand(&rx_buf[hdr_len], rx_payload_len, &latest_command);
            ROS2_ApplyCommand(&latest_command);
            last_cmd_tick = HAL_GetTick();
        }

        /* --- command timeout protection (only when ROS2 has taken over) --- */
        if (ros2_active && (HAL_GetTick() - last_cmd_tick) > ROS2_CMD_TIMEOUT_MS) {
            // 控制指令超时，切回停止状态

            // if (chassis_move.start_flag == 1) {
            //     chassis_move.start_flag = 0;
            //     chassis_move.v_set  = 0.0f;
            //     chassis_move.x_set  = chassis_move.x_filter;
            //     chassis_move.leg_set = 0.08f;
            //     chassis_move.roll_set = 0.0f;
            // }
            ros2_active = 0;
        }

        /* --- send telemetry --- */
        ROS2_SendTelemetry();

        osDelay(ROS2_TASK_PERIOD_MS);
    }
}
