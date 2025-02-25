#include "df_player.h"

uint8_t play_flag = 0;

//Função para cálculo checksum
static uint16_t calculate_checksum(DFPlayer_Command* cmd) 
{
    uint16_t sum = 0;
    sum = cmd->cmd + cmd->feedback + cmd->version + cmd->params + cmd->length;
    
    return 0xFFFF - sum + 1;
}

//Função para enviar comando para UART
static void dfplayer_send_cmd_uart(DFPlayer_Command* cmd_struct)
{
    //Clear circular buffer
    clear_buffer(DF_PLAYER_UART);
    //Envia bytes da struct via UART1 usando o buffer circular
    buffer_send_byte((uint8_t*) cmd_struct, sizeof(DFPlayer_Command));
}

//Função para enviar comando e parametros definidos pelo usuário
static void dfplayer_send_cmd(uint8_t cmd, uint8_t feedback, uint8_t param1, uint8_t param2)
{
    DFPlayer_Command command;
    command.start_byte = START_BYTE;
    command.version = VER_INFO;
    command.length = CMD_LEN;
    command.cmd = cmd;
    command.feedback = feedback;
    command.params = (param2 << 8) | (param1);
    uint16_t cs = calculate_checksum(&command);
    command.checksum_h = (cs >> 8) & 0xFF;
    command.checksum_l = cs & 0xFF; 
    command.end_byte = END_BYTE;

    dfplayer_send_cmd_uart(&command);
}

void df_player_init(uint8_t volume)
{
    //Setar parametros iniciais
    dfplayer_send_cmd(CMD_INIT_PARAMS, FEEDBACK_EN, 0x00, TF_CARD_SRC);
    delay_ms(200);
    //Setar volume
    dfplayer_send_cmd(CMD_SPEC_VOL, 0x00, 0x00, volume);
    delay_ms(500);
}

void df_player_play_first_track()
{
    dfplayer_send_cmd(CMD_SET_TRACK, 0x00, 0x00, 0x01);
    delay_ms(500);
}

void df_player_play_next_track()
{
    dfplayer_send_cmd(CMD_NEXT, 0x00, 0x00, 0x00);
    delay_ms(200);
}

void df_player_play_prev_track()
{
    dfplayer_send_cmd(CMD_PREV, 0x00, 0x00, 0x00);
    delay_ms(200);
}

void df_player_pause()
{
    dfplayer_send_cmd(CMD_PAUSE, 0x00, 0x00, 0x00);
    delay_ms(200);
}

void df_player_playback()
{
    dfplayer_send_cmd(CMD_PLAY, 0x00, 0x00, 0x00);
    delay_ms(200);
}

void df_player_repeat_track()
{
    dfplayer_send_cmd(CMD_REPEAT, 0x01, 0x00, START_REPEAT);
    delay_ms(200);
}

void df_player_play_pause()
{
    if(read_gpio(GPIOC, 13)) {
        while(read_gpio(GPIOC, 13));
        if(play_flag) {
            df_player_pause();
            play_flag = 0;
        } else {
            df_player_playback();
            play_flag = 1;
        }
    }
}
