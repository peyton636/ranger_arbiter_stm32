# -*- coding: utf-8 -*-
"""Regenerate Jetson_BLOBЭ��_v2.md as UTF-8 (no BOM)."""
from pathlib import Path

DOC = r"""# Jetson �� MCU BLOB ������Э�飨v2.0 �ݰ���

| Ԫ���� | ֵ |
|--------|-----|
| **Э��汾** | **v2.0-draft.5.1** |
| **�ĵ�����** | 2026-06-15 |
| **������** | RS232 115200 8N1��USART2����CAN ģʽ������ͬ PAYLOAD |
| **����** | ���ֽ����� **��� BE**��`#pragma pack(1)` |

---

## 0. �����װ������ҵ��֡���ã�

```text
[0xAB][VER=0x01][MSG_ID][SEQ][LEN_H][LEN_L][FRAG_IDX=0][FRAG_CNT=1][FLAGS=0][PAYLOAD...]
```

| �ֽ� | �ֶ� | ȡֵ |
|:--:|------|------|
| 0 | MAGIC | �̶� **0xAB** |
| 1 | VER | �̶� **0x01** |
| 2 | MSG_ID | ���±� |
| 3 | SEQ | 0~255 ���� |
| 4~5 | LEN | **= sizeof(struct)**��u16 BE |
| 6~8 | FRAG/CRC | Ĭ�ϲ���Ƭ���� CRC |
| 9~ | PAYLOAD | packed struct ԭ���ֽ� |

**��ʱ��ͬ���촫**��`0xA5` ����֡��0x107/0x108�����䣬�� [Jetson_RS232Э��.md](./Jetson_RS232Э��.md)��

### ֡�� timestamp_ms��ǿ�ƣ�

ÿһ�� wire struct **byte0~3 = timestamp_ms��u32 BE��**����ʾ���ͷ������ɡ�����������ʱ�̡�

| ���� | timestamp_ms |
|------|--------------|
| Jetson��MCU��0x01��0x10�� | Jetson �������� |
| MCU��Jetson������ MSG�� | MCU tick ms |

��� PING/offset��`t_jetson = mcu_timestamp_ms - offset_ms`��

| ���� | ���� |
|------|------|
| **֡ stamp** | struct �� 4B����֡����ʱ�̣�**������**�� |
| **· stamp** | �� sensor �� stamp_f~r��ĳ·�������仯ʱ�� |

### MSG_ID ����

| MSG_ID | ��� | ���� | struct | ���� |
|:--:|------|:----:|--------|:--:|
| **0x01** | �� ���� | Jetson��MCU | `agv_control_t` | 14 B |
| **0x02** | �� ���� | MCU��Jetson | `agv_motion_t` | 40 B |
| **0x06** | �� ���� | MCU��Jetson | `agv_motor04_t` | 44 B |
| **0x07** | �� ���� | MCU��Jetson | `agv_motor58_t` | 44 B |
| **0x08** | �� ���� | MCU��Jetson | `agv_energy_t` | 41 B |
| **0x0B** | �� ���� | MCU��Jetson | `agv_motor_pos_t` | 36 B����ѡ 1 Hz�� |
| **0x05** | �� GPS | MCU��Jetson | `gps_compact_t` | 32 B |
| **0x04** | �� ������ | MCU��Jetson | `sensor_blob_t` | 28 B |
| **0x10** | �� ������ | Jetson��MCU | `sensor_cfg_t` | 8 B |
| **0x03** | �� MCU | MCU��Jetson | `mcu_status_t` | 42 B |

---

## �� ����

MCU �ۺϵ��� CAN �� **4 ������֡��0x02/06/07/08���� ~40B��**��Jetson �·� **0x01** �� �ٲú�ת CAN��

### 1.1 ���� `agv_control_t`��MSG 0x01��14 B��

| | 0~3 | 4~5 | 6~7 | 8~9 | 10 | 11 | 12 | 13 |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **�ֶ�** | timestamp_ms | linear_vel | angular_vel | steer_angle | control_mode | motion_drive_info | clear_fault | light_info |

- **linear_vel / angular_vel / steer_angle**��s16 BE��mm/s��0.001 rad/s��0.001 rad
- **control_mode**��0x00 ������0x01 CAN ָ��
- **motion_drive_info**��bit[1:0] �˶�ģʽ��bit2 ����/��ѹ����
- **clear_fault**���� 0 �������� 0x441 ���
- **light_info**��bit0 ʹ�ܣ�bit1 ��ģʽ

### 1.2 Ϊ�β��� 201 B ��֡��

| �� | �ֽ� | ˵�� |
|----|:--:|------|
| motor[8]��16B | 128 | ռ 64%������ |
| odom+remote+bms | 37 | |
| �˶�+ϵͳ | 33 | |

**�Բ�**���� 4 ֡������� 10B �����壻`light_info+light_count` �ϲ�Ϊ `light_pack`��1B����

### 1.3 �˶� `agv_motion_t`��MSG 0x02��40 B��

| | 0~3 | 4 | 5 | 6 | 7~10 | 11~12 | 13~18 | 19~26 | 27~34 | 35~39 |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **�ֶ�** | ts | system_info | motion_info | light_pack | fault_code | bat_v | vel��3 | wheel_angle[4] | wheel_speed[4] | rsv |

- **system_info**��bit0 �쳣��bit[2:1] ����/CAN/ң��
- **motion_info**��bit[1:0] ������/б��/����/פ����bit2 �л��У�bit3 ����ģʽ
- **light_pack**��bit0~1 �ƣ�bit[7:2] ���� 0~63
- **fault_code**��u32 BE��0x211 ԭ��
- **bat_v**��u16����0.1 V

### 1.4 �����MSG 0x06 / 0x07���� 44 B��

| MSG | ��� | CAN |
|:---:|:--:|-----|
| 0x06 | 0~3 ���� | 0x251~254 |
| 0x07 | 4~7 ת�� | 0x255~258 |

**motor_compact_t��10 B/̨��**��speed s16��current s16��voltage u16��temp i8��driver_status u8 FLAG��position_lo u16��

**driver_status** ͬ���� 0x261 byte5��VLOW/MOT_OT/DRV_OC/DRV_OT/SENSOR/ERR/EN��

### 1.5 ��Դ `agv_energy_t`��MSG 0x08��41 B��

ts(4) + odom[4] s32(16) + bms(14) + remote[7]�����ڽ��� 50 ms��

### 1.6 ��ѡȫ���� `agv_motor_pos_t`��MSG 0x0B��36 B��1 Hz��

ts + position_pulse[8] u32��

### 1.7 ���ͽ���

ÿ 20 ms��0x02 �� 0x06 �� 0x07��ÿ 50 ms��0x08��ÿ 100 ms��0x05 GPS��

---

## �� GPS

### 2.1 `gps_compact_t`��MSG 0x05��32 B��

| | 0~3 | 4 | 5 | 6~7 | 8~9 | 10~13 | 14~17 | 18~19 | 20~21 | 22~25 | 26~31 |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **�ֶ�** | ts | flags | num_sv | hdop | speed | lat | lon | heading | alt | utc | rsv |

**flags**��bit0 POS��bit1 VEL��bit2 HEADING��bit3 FIX��bit4 USEFULL��

- hdop��100����Ч 0xFFFF��alt_dm ��Ч 0x7FFF��utc �� GPS �� 0��

---

## �� ������

### 3.1 `sensor_blob_t`��MSG 0x04��28 B��

| | 0~3 | 4~11 | 12~27 |
|:---:|:---:|:---:|
| **�ֶ�** | ts | dist_f/b/l/r u16 | stamp_f/b/l/r u32 |

| ȡֵ | ���� |
|:--:|------|
| 0~60000 | ��Ч���� mm |
| **0xFFFF** | ��Ч/��ʱ/δ֪ |

### 3.2 `sensor_cfg_t`��MSG 0x10��8 B��

ts + threshold_mm u16 + enable_mask u8��bit0~3 ǰ/��/��/��ʹ�ܣ���

---

## �� MCU

### 4.1 `mcu_status_t`��MSG 0x03��42 B��

| | 0~3 | 4 | 5 | 6 | 7 | 8~13 | 14~21 | 22~37 | 38~39 | 40 | 41 |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **�ֶ�** | ts | seq | safety | link | limit | arb_v/w/steer | sonar[4] | stamp[4] | near | jetson_seq | rsv |

- **safety**��0x01 ������0x02 ���٣�0x03 ������0x04 ����
- **link_flags**��bit0 ������ʧ��bit1 ���̹��ϣ�bit2 ������bit3 CAN ��Ч��bit4 UART ��Ч

#### sonar[4]��byte14~21��

�� **��3.1 dist_*** ��ͬ��ǰ/��/��/�� **mm**����Ч **0xFFFF**��

> **Ϊ�� 0x03 �ﻹ�� sonar��** �� 0x04 ������ͬ���� **MCU �ٲ��̱߳��ؿ���**��`DistSnapshot`��������/����ͣ���� 20 ms ����������ص� Jetson ����� 0x04 ֡��Jetson **���ı�������ֻ�� 0x04 ����**��0x03 ��� sonar ����Ϊ����/�����ֶΣ������汾��ɾ�԰� 0x03 ���� 22 B��

#### stamp[4]��byte22~37��

�� **��3.1 stamp_*** ��ͬ��ÿ· u32 BE��stable_mm ���仯ʱ�̡�

#### nearest_mm��byte38~39��

������С��Ч���� mm��ȫ��Ч **0xFFFF**��

---

## ��¼ A��C �ṹ��

```c
#pragma pack(push, 1)

typedef struct { /* 0x01, 14B */
    uint32_t timestamp_ms;
    int16_t  linear_vel, angular_vel, steer_angle;
    uint8_t  control_mode, motion_drive_info, clear_fault, light_info;
} agv_control_t;

typedef struct { /* 10B */
    int16_t  speed_rpm, current;
    uint16_t voltage;
    int8_t   temperature;
    uint8_t  driver_status;
    uint16_t position_lo;
} motor_compact_t;

typedef struct { /* 0x02, 40B */
    uint32_t timestamp_ms;
    uint8_t  system_info, motion_info, light_pack;
    uint32_t fault_code;
    uint16_t battery_voltage;
    int16_t  linear_velocity, angular_velocity, steering_angle;
    int16_t  wheel_angle[4], wheel_speed[4];
    uint8_t  rsv[5];
} agv_motion_t;

typedef struct { /* 0x06/0x07, 44B */
    uint32_t timestamp_ms;
    motor_compact_t motor[4];
} agv_motor04_t, agv_motor58_t;

typedef struct { /* 0x08, 41B */
    uint32_t timestamp_ms;
    int32_t  odom[4];
    struct {
        uint8_t bms_soc, bms_soh;
        uint16_t bms_voltage;
        int16_t  bms_current, bms_temperature;
        uint8_t  bms_alarm1, bms_alarm2, bms_warning1, bms_warning2;
    } bms;
    uint8_t remote[7];
} agv_energy_t;

typedef struct { /* 0x0B, 36B */
    uint32_t timestamp_ms;
    uint32_t position_pulse[8];
} agv_motor_pos_t;

typedef struct { /* 0x05, 32B */
    uint32_t timestamp_ms;
    uint8_t  flags, num_sv;
    uint16_t hdop_x100, speed_cms;
    int32_t  lat_e7, lon_e7;
    int16_t  heading_x100, alt_dm;
    uint32_t utc_sec;
    uint8_t  rsv[6];
} gps_compact_t;

typedef struct { /* 0x04, 28B */
    uint32_t timestamp_ms;
    uint16_t dist_mm[4];
    uint32_t stamp_ms[4];
} sensor_blob_t;

typedef struct { /* 0x10, 8B */
    uint32_t timestamp_ms;
    uint16_t threshold_mm;
    uint8_t  enable_mask, rsv;
} sensor_cfg_t;

typedef struct { /* 0x03, 42B */
    uint32_t timestamp_ms;
    uint8_t  seq, safety, link_flags, limit_factor;
    int16_t  arb_v, arb_w, arb_steer;
    uint16_t sonar_mm[4];
    uint32_t sonar_stamp_ms[4];
    uint16_t nearest_mm;
    uint8_t  jetson_seq, rsv;
} mcu_status_t;

#pragma pack(pop)
```

---

## ��¼ B����������

| MSG | ���� |
|:---:|:--:|
| 0x01 | 20 ms |
| 0x02/0x06/0x07 | 20 ms |
| 0x08 | 50 ms |
| 0x03/0x04 | 20 ms |
| 0x05 | 100 ms |

---

## ��¼ C�������¼

| �汾 | ���� | ���� |
|------|------|------|
| v2.0-draft.5 | 2026-06-15 | 201B ��Ϊ 0x02/06/07/08����� 10B |
| v2.0-draft.5.1 | 2026-06-15 | �޸� GBK/UTF-8 ���룻���� 0x03 sonar ���� |
"""

def main():
    root = Path(__file__).resolve().parents[1]
    for name in ("Jetson_BLOBЭ��_v2.md", "Jetson_BLOB_protocol_v2.md"):
        p = root / "docs" / name
        p.write_text(DOC, encoding="utf-8", newline="\n")
        print("wrote", p, "lines", DOC.count("\n") + 1)

if __name__ == "__main__":
    main()
