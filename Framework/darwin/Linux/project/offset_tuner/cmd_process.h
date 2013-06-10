#ifndef _DXL_MANAGER_CMD_PROCESS_H_
#define _DXL_MANAGER_CMD_PROCESS_H_


#include "LinuxDARwIn.h"


#define PROGRAM_VERSION		"v1.00"
#define SCREEN_COL			80
#define SCREEN_ROW			22

// Position of Column
#define CMD_COL			2
#define GOAL_COL		21
#define OFFSET_COL		28
#define MODVAL_COL		35
#define PRSVAL_COL		42
#define ERRORS_COL		49
#define P_GAIN_COL      56
#define I_GAIN_COL      63
#define D_GAIN_COL      70


// Position of Row
#define ID_1_ROW	0
#define ID_2_ROW	1
#define ID_3_ROW	2
#define ID_4_ROW	3
#define ID_5_ROW	4
#define ID_6_ROW	5
#define ID_7_ROW	6
#define ID_8_ROW	7
#define ID_9_ROW	8
#define ID_10_ROW	9
#define ID_11_ROW	10
#define ID_12_ROW	11
#define ID_13_ROW	12
#define ID_14_ROW	13
#define ID_15_ROW	14
#define ID_16_ROW	15
#define ID_17_ROW	16
#define ID_18_ROW	17
#define ID_19_ROW	18
#define ID_20_ROW	19
#define CMD_ROW		21


int _getch();
bool AskSave();


// Move cursor
void GoToCursor(int col, int row);
void MoveUpCursor();
void MoveDownCursor();
void MoveLeftCursor();
void MoveRightCursor();

// Disp & Drawing
void DrawIntro(Robot::CM730 *cm730);
void DrawEnding();
void DrawPage();
void DrawStep(int index);
void DrawStepLine(bool erase);
void ClearCmd();
void PrintCmd(const char *message);

// Edit value
void UpDownValue(Robot::CM730 *cm730, int offset);
void SetValue(Robot::CM730 *cm730, int value);
int GetValue();
void ToggleTorque(Robot::CM730 *cm730);

// Command process
void BeginCommandMode();
void EndCommandMode();
void HelpCmd();
void OnOffCmd(Robot::CM730 *cm730, bool on, int num_param, int *list);
void SaveCmd(minIni *ini);
void GainCmd(Robot::CM730 *cm730, int value, int pid_col);

void ReadStep(Robot::CM730 *cm730);

#endif
