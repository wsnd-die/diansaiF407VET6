#ifndef __VOICE_H
#define __VOICE_H

#include "main.h"

extern char Apple[];
extern char Pear[];
extern char Pumpkin[];
extern char Pepper[];
extern char Tomato[];
extern char Eggplant[];
extern char Teaminfor[];
extern char Mature[];
extern char Inmature[];
extern char Bad[];
extern char Music[];
extern char num1[];
extern char num2[];
extern char num3[];
extern char num4[];
extern char num5[];
extern char num6[];
extern char num7[];
extern char num8[];
extern char num9[];
extern char num10[];

typedef enum {
    OverallCycle   = 0x00,
    SingleCycle    = 0x01,
    SingleStop     = 0x02,
    OverallRandom  = 0x03,
    CatalogCycle   = 0x04,
    CatalogRandom  = 0x05,
    CatalogTurnPlay = 0x06,
    OverallTurnPlay = 0x07,
} LoopModeSelect;

typedef enum {
    CheckPlayState = 0x01,
    Play = 0x02,
    Pause = 0x03,
    Stop = 0x04,
    LastSong = 0x05,
    NextSong = 0x06,
    CheckOnelineDisksign = 0x09,
    CheckCurrentDisksign = 0x0A,
    CheckTotalTrack = 0x0C,
    CurrentTrack = 0x0D,
    LastFloder = 0x0E,
    NextFloder = 0x0F,
    EndPlay = 0x10,
    CheckFloderFirstTrack = 0x11,
    CheckFloderAllTrack = 0x12,
    AddVolume = 0x14,
    DecVolume = 0x15,
    EndZHPlay = 0x1C,
    CheckSongShortName = 0x1E,
    EndLoop = 0x21,
    GetTotalSongTime = 0x24,
    OpenPlayTime = 0x25,
    ClosePlayTime = 0x26,
} UartCommand;

typedef enum {
    AppointTrack = 0x07,
    SetCycleCount = 0x19,
    SetEQ = 0x1A,
    SelectTrackNoPlay = 0x19,
    GoToDisksign = 0x0B,
    SetVolume = 0x13,
    SetLoopMode = 0x18,
    SetChannel = 0x1D,
    AppointTimeBack = 0x22,
    AppointTimeFast = 0x23,
} UartCommandData;

typedef enum {
    JQ8X00_USB = 0x00,
    JQ8X00_SD = 0x01,
    JQ8X00_FLASH = 0x02,
} JQ8X00_Symbol;

void playFruitSound(int quantity, char *Fruitname);
void JQ8x00_Command(UartCommand Command);
void JQ8x00_Command_Data(UartCommandData Command, uint8_t DATA);
void JQ8x00_RandomPathPlay(JQ8X00_Symbol symbol, char *DATA);
void JQ8x00_RandomPathPlay_Inserch(JQ8X00_Symbol symbol, char *DATA);
void JQ8x00_Volumn(uint8_t vol);
void JQ8x00_Over(void);
void JQ8x00_Stop(void);
void JQ8x00_Hold(void);
void JQ8x00_Loop(void);
void JQ8x00_Loop_times2(void);
void Voice_Num(int Num);

#endif
