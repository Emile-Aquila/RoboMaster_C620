/*
 * CAN_C620_System.c
 *
 *  Created on: 7 8, 2023
 *      Author: Emile
 */


#include "CAN_C620_System.h"
#include "CAN_C620_Def.h"
#include <stdio.h>
#include <stdint.h>

typedef struct {
    uint32_t StdId; // 18bit
    uint32_t DLC;
    uint8_t bytes[8];
} CANTxBuf;

typedef struct {
    CANTxBuf buffer[CAN_TXBUFFER_SIZE];
    uint32_t read_point;
    uint32_t write_point;
    uint8_t is_full;
} CAN_RingBuf;


CAN_HandleTypeDef *_c620_phcan_global;  // 変更しない事
CAN_RingBuf _can_buf_ring1 = {{}, 0, 0, 0};
c620_feedback_data_raw _c620_feedback_data_raw_global[9];


HAL_StatusTypeDef _C620_PushTx8Bytes(CAN_RingBuf *p_can_ring, uint32_t StdId, uint8_t *bytes, uint32_t size) {
    p_can_ring->buffer[p_can_ring->write_point].DLC = size;
    p_can_ring->buffer[p_can_ring->write_point].StdId = StdId;
    for (uint8_t i = 0; i < size; i++)p_can_ring->buffer[p_can_ring->write_point].bytes[i] = bytes[i];

    if (p_can_ring->is_full == 1) {
        p_can_ring->read_point = ((p_can_ring->read_point) + 1) & (CAN_TXBUFFER_SIZE - 1);
    }
    p_can_ring->write_point = ((p_can_ring->write_point) + 1) & (CAN_TXBUFFER_SIZE - 1);

    if (p_can_ring->write_point == p_can_ring->read_point) {
        p_can_ring->is_full = 1;
    }
    return HAL_OK;
}

HAL_StatusTypeDef _C620_PopSendTx8Bytes(CAN_HandleTypeDef *phcan, CAN_RingBuf *p_can_ring) {
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;

    txHeader.RTR = CAN_RTR_DATA;
    txHeader.IDE = CAN_ID_STD;
    txHeader.TransmitGlobalTime = DISABLE;
    while (HAL_CAN_GetTxMailboxesFreeLevel(phcan) > 0) {
        if ((p_can_ring->is_full == 0) && (p_can_ring->read_point == p_can_ring->write_point))break;

        txHeader.DLC = p_can_ring->buffer[p_can_ring->read_point].DLC;
        txHeader.StdId = p_can_ring->buffer[p_can_ring->read_point].StdId;
        txHeader.ExtId = 0;

        HAL_StatusTypeDef ret = HAL_CAN_AddTxMessage(phcan, &txHeader, p_can_ring->buffer[p_can_ring->read_point].bytes,
                                                     &txMailbox);
        if (ret != HAL_OK)return ret;
        p_can_ring->read_point = ((p_can_ring->read_point) + 1) & (CAN_TXBUFFER_SIZE - 1);
        p_can_ring->is_full = 0;
    }
    return HAL_OK;
}


HAL_StatusTypeDef C620_SendBytes(CAN_HandleTypeDef *phcan, uint32_t StdId, uint8_t *bytes, uint32_t size) { // 命令を送信する関数
    uint32_t quotient = size / 8;
    uint32_t remainder = size - (8 * quotient);
    HAL_StatusTypeDef ret;

    for (uint8_t i = 0; i < quotient; i++) {
        ret = _C620_PushTx8Bytes(&_can_buf_ring1, StdId, bytes + i * 8, 8);
        if (ret != HAL_OK) {
            Error_Handler();
            return ret;
        }
    }

    if (remainder != 0) {
        ret = _C620_PushTx8Bytes(&_can_buf_ring1, StdId, bytes + quotient * 8, remainder);
        if (ret != HAL_OK) {
            Error_Handler();
            return ret;
        }
    }
    ret = _C620_PopSendTx8Bytes(phcan, &_can_buf_ring1);
    if (ret != HAL_OK) {
        Error_Handler();
        return ret;
    }
    return HAL_OK;
}


void C620_WhenTxMailboxCompleteCallbackCalled(CAN_HandleTypeDef *phcan) {
    if (_c620_phcan_global != phcan)return;
    _C620_PopSendTx8Bytes(phcan, &_can_buf_ring1);
}

void C620_WhenTxMailboxAbortCallbackCalled(CAN_HandleTypeDef *phcan) {
    if (_c620_phcan_global != phcan)return;
    _C620_PopSendTx8Bytes(phcan, &_can_buf_ring1);
}


void _set_fb_data_raw(const uint8_t rxData[], uint8_t device_id) {
    if (device_id > 9 || device_id <= 0)return;

    _c620_feedback_data_raw_global[device_id]._get_counter += 1;
    if (_c620_feedback_data_raw_global[device_id]._get_counter > 128) {
        _c620_feedback_data_raw_global[device_id]._get_counter = 128;  // overflow対策
    }

    if (_c620_feedback_data_raw_global[device_id]._get_counter < 20) {  // M3508のEncoderの初期位置を取得
        _c620_feedback_data_raw_global[device_id]._internal_offset_pos = (uint16_t) (rxData[0] << 8 | rxData[1]);
        _c620_feedback_data_raw_global[device_id].pos_pre = (uint16_t) (rxData[0] << 8 | rxData[1]);
        return;
    }

    // dataの設定
    _c620_feedback_data_raw_global[device_id].pos_pre = _c620_feedback_data_raw_global[device_id].pos;
    _c620_feedback_data_raw_global[device_id].pos = (uint16_t) (rxData[0] << 8 | rxData[1]);
    _c620_feedback_data_raw_global[device_id].vel = (int16_t) (rxData[2] << 8 | rxData[3]);
    _c620_feedback_data_raw_global[device_id].cur = (int16_t) (rxData[4] << 8 | rxData[5]);

    // 回転数の計算
    int32_t diff_pos = (int32_t) (_c620_feedback_data_raw_global[device_id].pos) -
                       (int32_t) (_c620_feedback_data_raw_global[device_id].pos_pre);
    if (diff_pos > 4096) {
        if (_c620_feedback_data_raw_global[device_id]._rot_num != -(INT64_MAX / 10)) {
            _c620_feedback_data_raw_global[device_id]._rot_num -= 1;
        }  // overflow対策
    } else if (diff_pos < -4096) {
        if (_c620_feedback_data_raw_global[device_id]._rot_num != (INT64_MAX / 10)) {
            _c620_feedback_data_raw_global[device_id]._rot_num += 1;
        }  // overflow対策
    }
}

void C620_WhenCANRxFifo1MsgPending(CAN_HandleTypeDef *phcan) {
    // Fifo0はCANLibで使うので、Fifo1を使う事。
    if (_c620_phcan_global != phcan)return;
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    if (HAL_CAN_GetRxMessage(phcan, CAN_RX_FIFO1, &rxHeader, rxData) != HAL_OK) {
        // Reception Error
        printf("GetRxMessage error\n\r");
        Error_Handler();
    }

    if (((rxHeader.StdId - 0x200) < 9) && ((rxHeader.StdId - 0x200) >= 0)) {
        _set_fb_data_raw(rxData, rxHeader.StdId - 0x200);  // fb_data_rawにデータを入力
    }
}


void Init_C620_CAN_System(CAN_HandleTypeDef *phcan) {  //CAN初期化
    _c620_phcan_global = phcan;
    CAN_FilterTypeDef sFilterConfig;

    //フィルタバンク設定
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    // CAN2をつかうならSlaveStartFilterBank以上の値をFilterBankに設定する必要がある
    // FIFO1に初期化用のフィルタを設定
    sFilterConfig.FilterBank = 10; // CANLibで6まで使ってる
    sFilterConfig.FilterIdHigh = (0x200 | 0b1000) << 5;
    sFilterConfig.FilterMaskIdHigh = ((1 << 12) - 1) << 5;
    sFilterConfig.FilterIdLow = 0b000; // 下16bit
    sFilterConfig.FilterMaskIdLow = (1 << 16) - 1;  // Standard ID
    if (HAL_CAN_ConfigFilter(phcan, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }

    sFilterConfig.FilterBank = 11; // CANLibで6まで使ってる
    sFilterConfig.FilterIdHigh = (0x200 | 0b0001) << 5;
    sFilterConfig.FilterMaskIdHigh = (((1 << 12) - 1) ^ 0b0110) << 5;
    sFilterConfig.FilterIdLow = 0b000; // 下16bit
    sFilterConfig.FilterMaskIdLow = (1 << 16) - 1;  // Standard ID
    if (HAL_CAN_ConfigFilter(phcan, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }

    sFilterConfig.FilterBank = 12; // CANLibで6まで使ってる
    sFilterConfig.FilterIdHigh = (0x200 | 0b0010) << 5;
    sFilterConfig.FilterMaskIdHigh = (((1 << 12) - 1) ^ 0b0100) << 5;
    sFilterConfig.FilterIdLow = 0b000; // 下16bit
    sFilterConfig.FilterMaskIdLow = (1 << 16) - 1;  // Standard ID
    if (HAL_CAN_ConfigFilter(phcan, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }

    sFilterConfig.FilterBank = 13; // CANLibで6まで使ってる
    sFilterConfig.FilterIdHigh = (0x200 | 0b0100) << 5;
    sFilterConfig.FilterMaskIdHigh = ((1 << 12) - 1) << 5;
    sFilterConfig.FilterIdLow = 0b000; // 下16bit
    sFilterConfig.FilterMaskIdLow = (1 << 16) - 1;  // Standard ID
    if (HAL_CAN_ConfigFilter(phcan, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_CAN_Start(phcan) != HAL_OK) {
        printf(" -> Start Error CAN_C620\n");
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(phcan, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK) {
        printf(" -> FIFO1 CAN_Activation error1\n\r");
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(phcan, CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK) {
        printf(" -> CAN_Activation error2\n\r");
        Error_Handler();
    }

    for (uint8_t i = 0; i < 9; i++) {  // init fb_data_raw
        _c620_feedback_data_raw_global[i].pos = 0;
        _c620_feedback_data_raw_global[i].pos_pre = 0;
        _c620_feedback_data_raw_global[i]._rot_num = 0;
        _c620_feedback_data_raw_global[i].vel = 0;
        _c620_feedback_data_raw_global[i].cur = 0;
        _c620_feedback_data_raw_global[i]._get_counter = 0;
        _c620_feedback_data_raw_global[i]._internal_offset_pos = 0;
    }
}


C620_FeedbackData Get_C620_FeedbackData(C620_DeviceInfo *device_info) {
    uint8_t device_id = device_info->device_id;
    if (device_id >= 9)device_id = 0;

    C620_FeedbackData fb_data;
    fb_data.device_id = device_id;
    c620_feedback_data_raw *data = &(_c620_feedback_data_raw_global[device_id]);

    int16_t offset_pos = (int16_t) (data->pos) - (int16_t) (data->_internal_offset_pos);
    if (device_info->ctrl_param.use_internal_offset == C620_USE_OFFSET_POS_ENABLE) {
        fb_data.position = ((float) (offset_pos) * 6.28318f) / 8192.0f + (float) (data->_rot_num) * 6.28318f;
    } else {
        fb_data.position = ((float) (data->pos) * 6.28318f) / 8192.0f + (float) (data->_rot_num) * 6.28318f;
    }
    fb_data.velocity = ((float) (data->vel)) * 3.141592f / 60.0f;
    fb_data.current = ((float) (data->cur * 20)) / 16384.0f;
    fb_data.get_flag = (data->_get_counter > 20);

    return fb_data;
}



