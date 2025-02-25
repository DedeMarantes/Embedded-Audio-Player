#ifndef DF_PLAYER_H
#define DF_PLAYER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "uart.h"
#include "core_drivers.h"
//Constantes
#define START_BYTE      0x7E
#define END_BYTE        0xEF
#define CMD_LEN         0x06
#define FEEDBACK_EN     0x01
#define VER_INFO        0xFF
//Comandos com base no datasheet do DFPlayer Mini
#define CMD_NEXT        0x01
#define CMD_PREV        0x02
#define CMD_SET_TRACK   0x03
#define CMD_INC_VOL     0x04
#define CMD_DEC_VOL     0x05
#define CMD_SPEC_VOL     0x06
#define CMD_SET_EQ      0x07 //especificar modo eq
#define CMD_SET_PLAYBCK 0x08 //especificar modo de playback
#define CMD_SET_SRC     0x09 //especificar source do playback
#define CMD_STANDBY     0x0A  
#define CMD_NORMAL      0x0B  
#define CMD_RESET       0x0C  
#define CMD_PLAY        0x0D  
#define CMD_PAUSE       0x0E  
#define CMD_PLAY_FOLDER 0x0F  
#define CMD_SET_VOL     0x10  
#define CMD_REPEAT      0x11  
#define CMD_INIT_PARAMS 0x3F

//Constantes parâmetros
#define TF_CARD_SRC     0x02
#define START_REPEAT    0x01

//Tipos de parâmetros usados por alguns comandos
typedef enum {
    NORMAL = 0,
    POP,
    ROCK,
    JAZZ,
    CLASSIC,
    BASE
} EqModeType;

typedef enum {
    REPEAT = 0,
    FOLDER_REPEAT,
    SINGLE_REPEAT,
    RANDOM
} PlaybackModeType;

typedef enum {
    USB = 0,
    SD_CARD,
    AUX,
    SLEEP,
    FLASH_SRC
} PlaybackSourceType;

//Struct do comando
typedef struct __attribute__((packed)) {
    uint8_t start_byte;
    uint8_t version;
    uint8_t length;
    uint8_t cmd;
    uint8_t feedback;
    uint16_t params;
    uint8_t checksum_h;
    uint8_t checksum_l;
    uint8_t end_byte;
} DFPlayer_Command;

void df_player_init(uint8_t volume);
void df_player_play_first_track();
void df_player_playback();
void df_player_pause();
void df_player_play_prev_track();
void df_player_play_next_track();
void df_player_repeat_track();
void df_player_play_pause();



#ifdef __cplusplus
}
#endif

#endif