#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <term.h>
#include <fcntl.h>
#include <ncurses.h>
#include "cmd_process.h"

using namespace Robot;


int Col = OFFSET_COL;
int Row = ID_1_ROW;
int Old_Col;
int Old_Row;
bool bBeginCommandMode = false;
bool bEdited = false;
int indexPage = 1;
Action::PAGE Page;
Action::STEP Step;
Action::STEP PresentPos;

int P_Gain[JointData::NUMBER_OF_JOINTS];
int I_Gain[JointData::NUMBER_OF_JOINTS];
int D_Gain[JointData::NUMBER_OF_JOINTS];

//                           1     2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20
int InitPose[21] = {2047, 1480, 2610, 1747, 2343, 2147, 1944, 2047, 2047, 2047, 2047, 2013, 2080, 2047, 2047, 2063, 2030, 2047, 2047, 2047, 2170};

int _getch()
{
	struct termios oldt, newt;
	int ch;
	tcgetattr( STDIN_FILENO, &oldt );
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr( STDIN_FILENO, TCSANOW, &newt );
	ch = getchar();
	tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
	return ch;
}

int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}

struct termios oldterm, new_term;
void set_stdin(void)
{
	tcgetattr(0,&oldterm);
	new_term = oldterm;
	new_term.c_lflag &= ~(ICANON | ECHO | ISIG); // 의미는 struct termios를 찾으면 됨.
	new_term.c_cc[VMIN] = 1;
	new_term.c_cc[VTIME] = 0;
	tcsetattr(0, TCSANOW, &new_term);
}

void reset_stdin(void)
{
	tcsetattr(0, TCSANOW, &oldterm);
}

void ReadStep(CM730 *cm730)
{
	int value;
	for(int id=0; id<31; id++)
	{
		if(id >= JointData::ID_R_SHOULDER_PITCH && id <= JointData::ID_HEAD_TILT)
		{
			if(cm730->ReadByte(id, MX28::P_TORQUE_ENABLE, &value, 0) == CM730::SUCCESS)
			{
				if(value == 1)
				{
				    unsigned char table[50];
				    if(cm730->ReadTable(id, MX28::P_P_GAIN, MX28::P_PRESENT_POSITION_H, table, 0) == CM730::SUCCESS)
				    {
				        Step.position[id] = cm730->MakeWord(table[MX28::P_GOAL_POSITION_L], table[MX28::P_GOAL_POSITION_H]);
				        PresentPos.position[id] = cm730->MakeWord(table[MX28::P_PRESENT_POSITION_L], table[MX28::P_PRESENT_POSITION_H]);
				        P_Gain[id] = table[MX28::P_P_GAIN];
				        I_Gain[id] = table[MX28::P_I_GAIN];
				        D_Gain[id] = table[MX28::P_D_GAIN];
				    }
					//if(cm730->ReadWord(id, MX28::P_GOAL_POSITION_L, &value, 0) == CM730::SUCCESS)
					//	Step.position[id] = value;
					else
						Step.position[id] = Action::INVALID_BIT_MASK;
				}
				else
					Step.position[id] = Action::TORQUE_OFF_BIT_MASK;
			}
			else
				Step.position[id] = Action::INVALID_BIT_MASK;
		}
		else
			Step.position[id] = Action::INVALID_BIT_MASK;
	}
}

bool AskSave()
{
	if(bEdited == true)
	{
		PrintCmd("Are you sure? (y/n)");
		if(_getch() != 'y')
		{
			ClearCmd();
			return true;
		}
	}

	return false;
}


void GoToCursor(int col, int row)
{
	char *cursor;
	char *esc_sequence;
	cursor = tigetstr("cup");
	esc_sequence = tparm(cursor, row, col);
	putp(esc_sequence);

	Col = col;
	Row = row;
}

void MoveUpCursor()
{
	if(Col >= GOAL_COL && Col <= D_GAIN_COL)
	{
		if( Row > ID_1_ROW )
			GoToCursor(Col, Row-1);
	}
}

void MoveDownCursor()
{
	if(Col >= GOAL_COL && Col <= D_GAIN_COL)
	{
		if( Row < ID_20_ROW )
			GoToCursor(Col, Row+1);
	}
}

void MoveLeftCursor()
{
	switch(Col)
	{
	case OFFSET_COL:
		GoToCursor(GOAL_COL, Row);
		break;

	case MODVAL_COL:
		GoToCursor(OFFSET_COL, Row);
		break;

	case PRSVAL_COL:
		GoToCursor(MODVAL_COL, Row);
		break;

	case ERRORS_COL:
		GoToCursor(PRSVAL_COL, Row);
		break;

    case P_GAIN_COL:
        GoToCursor(ERRORS_COL, Row);
        break;

    case I_GAIN_COL:
        GoToCursor(P_GAIN_COL, Row);
        break;

	case D_GAIN_COL:
        GoToCursor(I_GAIN_COL, Row);
        break;
	}
}

void MoveRightCursor()
{
	switch(Col)
	{
	case GOAL_COL:
		GoToCursor(OFFSET_COL, Row);
		break;

	case OFFSET_COL:
		GoToCursor(MODVAL_COL, Row);
		break;

	case MODVAL_COL:
		GoToCursor(PRSVAL_COL, Row);
		break;

	case PRSVAL_COL:
		GoToCursor(ERRORS_COL, Row);
		break;

    case ERRORS_COL:
        GoToCursor(P_GAIN_COL, Row);
        break;

    case P_GAIN_COL:
        GoToCursor(I_GAIN_COL, Row);
        break;

    case I_GAIN_COL:
        GoToCursor(D_GAIN_COL, Row);
        break;
	}
}

void DrawIntro(CM730 *cm730)
{
    int n = 0;
    int param[JointData::NUMBER_OF_JOINTS * 5];
    int wGoalPosition, wStartPosition, wDistance;

    for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
    {
        wStartPosition = MotionStatus::m_CurrentJoints.GetValue(id);
        wGoalPosition = InitPose[id] + MotionManager::GetInstance()->m_Offset[id];
        if( wStartPosition > wGoalPosition )
            wDistance = wStartPosition - wGoalPosition;
        else
            wDistance = wGoalPosition - wStartPosition;

        wDistance >>= 3;
        if( wDistance < 8 )
            wDistance = 8;

        param[n++] = id;
        param[n++] = CM730::GetLowByte(wGoalPosition);
        param[n++] = CM730::GetHighByte(wGoalPosition);
        param[n++] = CM730::GetLowByte(wDistance);
        param[n++] = CM730::GetHighByte(wDistance);
    }

    cm730->SyncWrite(MX28::P_GOAL_POSITION_L, 5, JointData::NUMBER_OF_JOINTS - 1, param);

	int nrows, ncolumns;
    setupterm(NULL, fileno(stdout), (int *)0);
    nrows = tigetnum("lines");
    ncolumns = tigetnum("cols");

	system("clear");
	printf("\n");
	printf("[Offset Tuner for DARwIn %s]\n", PROGRAM_VERSION);
	printf("\n");
	printf(" *Terminal screen size must be %d(col)x%d(row).\n", SCREEN_COL, SCREEN_ROW);
    printf(" *Current terminal has %d columns and %d rows.\n", ncolumns, nrows);
	printf("\n");
	printf("\n");
	printf("Press any key to start program...\n");
	_getch();

	Action::GetInstance()->LoadPage(indexPage, &Page);

	ReadStep(cm730);
	Step.pause = 0;
	Step.time = 0;

	DrawPage();
}

void DrawEnding()
{
	system("clear");
	printf("\n");
	printf("Terminate Offset tuner");
	printf("\n");
}

void DrawPage()
{
	int old_col = Col;
	int old_row = Row;

    system("clear");
    // 80    01234567890123456789012345678901234567890123456789012345678901234567890123456789     //24
    printf( "ID: 1(R_SHO_PITCH)  [    ]        [    ]|                                      \n" );//0
    printf( "ID: 2(L_SHO_PITCH)  [    ]        [    ]|                                      \n" );//1
    printf( "ID: 3(R_SHO_ROLL)   [    ]        [    ]|                                      \n" );//2
    printf( "ID: 4(L_SHO_ROLL)   [    ]        [    ]|                                      \n" );//3
    printf( "ID: 5(R_ELBOW)      [    ]        [    ]|                                      \n" );//4
    printf( "ID: 6(L_ELBOW)      [    ]        [    ]|                                      \n" );//5
    printf( "ID: 7(R_HIP_YAW)    [    ]        [    ]|                                      \n" );//6
    printf( "ID: 8(L_HIP_YAW)    [    ]        [    ]|                                      \n" );//7
    printf( "ID: 9(R_HIP_ROLL)   [    ]        [    ]|                                      \n" );//8
    printf( "ID:10(L_HIP_ROLL)   [    ]        [    ]|                                      \n" );//9
    printf( "ID:11(R_HIP_PITCH)  [    ]        [    ]|                                      \n" );//0
    printf( "ID:12(L_HIP_PITCH)  [    ]        [    ]|                                      \n" );//1
    printf( "ID:13(R_KNEE)       [    ]        [    ]|                                      \n" );//2
    printf( "ID:14(L_KNEE)       [    ]        [    ]|                                      \n" );//3
    printf( "ID:15(R_ANK_PITCH)  [    ]        [    ]|                                      \n" );//4
    printf( "ID:16(L_ANK_PITCH)  [    ]        [    ]|                                      \n" );//5
    printf( "ID:17(R_ANK_ROLL)   [    ]        [    ]|                                      \n" );//6
    printf( "ID:18(L_ANK_ROLL)   [    ]        [    ]|                                      \n" );//7
    printf( "ID:19(HEAD_PAN)     [    ]        [    ]|                                      \n" );//8
    printf( "ID:20(HEAD_TILT)    [    ]        [    ]|                                      \n" );//9
    printf( "                     GOAL  OFFSET MODVAL PRSPOS ERRORS P_GAIN I_GAIN D_GAIN    \n" );//0
    printf( "]                                                                              " );  //1

	for(int i=0; i<=7; i++ )
		DrawStep(i);

	GoToCursor(old_col, old_row);
}

void DrawStep(int index)
{
	int old_col = Col;
	int old_row = Row;
	int col;

	switch(index)
	{
	case 0:
		col = OFFSET_COL;
	    for( int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++ )
	    {
            GoToCursor(col, id -1);
	        printf("%4d ", MotionManager::GetInstance()->m_Offset[id]);
	    }
		break;

	case 1:
		col = MODVAL_COL;
        for( int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++ )
        {
            GoToCursor(col, id -1);
            printf("%4d", Step.position[id]);
        }
		break;

	case 2:
		col = PRSVAL_COL;
        for( int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++ )
        {
            GoToCursor(col, id -1);
            if(PresentPos.position[id] & Action::INVALID_BIT_MASK)
                printf("----");
            else if(PresentPos.position[id] & Action::TORQUE_OFF_BIT_MASK)
                printf("????");
            else
                printf("%.4d", PresentPos.position[id]);
        }
        break;

	case 3:
		col = ERRORS_COL;
        for( int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++ )
        {
            GoToCursor(col, id -1);
            if(PresentPos.position[id] & Action::INVALID_BIT_MASK)
                printf("----");
            else if(PresentPos.position[id] & Action::TORQUE_OFF_BIT_MASK)
                printf("????");
            else
                printf("%4d ", PresentPos.position[id] - Step.position[id]);
        }
		break;

	case 4:
		col = GOAL_COL;
        for( int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++ )
        {
            GoToCursor(col, id -1);
            printf("%4d", InitPose[id]);
        }
		break;

	case 5:
	    col = P_GAIN_COL;
        for( int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++ )
        {
            GoToCursor(col, id -1);
            printf("%4d", P_Gain[id]);
        }
	    break;

    case 6:
        col = I_GAIN_COL;
        for( int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++ )
        {
            GoToCursor(col, id -1);
            printf("%4d", I_Gain[id]);
        }
        break;

    case 7:
        col = D_GAIN_COL;
        for( int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++ )
        {
            GoToCursor(col, id -1);
            printf("%4d", D_Gain[id]);
        }
        break;

	default:
		return;
	}

	GoToCursor( old_col, old_row );
}

void DrawStepLine(bool erase)
{
	int old_col = Col;
	int old_row = Row;
	int col;

	switch(Page.header.stepnum)
	{
	case 0:
		col = OFFSET_COL;
		break;

	case 1:
		col = MODVAL_COL;
		break;

	case 2:
		col = PRSVAL_COL;
		break;

	case 3:
		col = ERRORS_COL;
		break;

	default:
		return;
	}
	col--;

	for( int id=JointData::ID_R_SHOULDER_PITCH; id<(JointData::NUMBER_OF_JOINTS); id++ )
	{
		GoToCursor(col, id - 1);
		if(erase == true)
			printf( " " );
		else
			printf( "|" );
	}

	GoToCursor(old_col, old_row);
}

void ClearCmd()
{
	PrintCmd("");
}

void PrintCmd(const char *message)
{
	int len = strlen(message);
	GoToCursor(0, CMD_ROW);

	printf( "] %s", message);
	for(int i=0; i<(SCREEN_COL - (len + 2)); i++)
		printf(" ");

	GoToCursor(len + 2, CMD_ROW);
}

void UpDownValue(CM730 *cm730, int offset)
{
    if(Col == OFFSET_COL)
    {
        if((InitPose[Row + 1] + GetValue() + offset) > MX28::MAX_VALUE) SetValue(cm730, MX28::MAX_VALUE - InitPose[Row + 1]);
        else if((InitPose[Row + 1] + GetValue() + offset) < MX28::MIN_VALUE) SetValue(cm730, MX28::MIN_VALUE - InitPose[Row + 1]);
        else SetValue(cm730, GetValue() + offset);
        bEdited = true;
    }
    else
        SetValue(cm730, GetValue() + offset);
}

int GetValue()
{
	int col;
	int row;
	if(bBeginCommandMode == true)
	{
		col = Old_Col;
		row = Old_Row;
	}
	else
	{
		col = Col;
		row = Row;
	}

	if( col == GOAL_COL )
        return InitPose[row + 1];
	else if( col == OFFSET_COL )
        return MotionManager::GetInstance()->m_Offset[row + 1];
    else if( col == MODVAL_COL )
        return Step.position[row + 1];
    else if( col == PRSVAL_COL )
        return PresentPos.position[row + 1];
    else if( col == ERRORS_COL )
        return PresentPos.position[row + 1] - Step.position[row + 1];
    else if( col == P_GAIN_COL )
        return P_Gain[row + 1];
    else if( col == I_GAIN_COL )
        return I_Gain[row + 1];
    else if( col == D_GAIN_COL )
        return D_Gain[row + 1];

	return -1;
}

void SetValue(CM730 *cm730, int value)
{
	int col;
	int row;
	if(bBeginCommandMode == true)
	{
		col = Old_Col;
		row = Old_Row;
	}
	else
	{
		col = Col;
		row = Row;
	}

	GoToCursor(col, row);

	if( col == GOAL_COL )
	{
        if(value+MotionManager::GetInstance()->m_Offset[row + 1] >= 0 && value+MotionManager::GetInstance()->m_Offset[row + 1] <= MX28::MAX_VALUE)
        {
            if(!(Step.position[row + 1] & Action::INVALID_BIT_MASK) && !(Step.position[row + 1] & Action::TORQUE_OFF_BIT_MASK))
            {
                int error;
                if(cm730->WriteWord(row + 1, MX28::P_GOAL_POSITION_L, value+MotionManager::GetInstance()->m_Offset[row + 1], &error) == CM730::SUCCESS)
                {
                    if(!(error & CM730::ANGLE_LIMIT))
                    {
                        InitPose[row + 1] = value;
                        printf( "%.4d", value );
                        Step.position[row + 1] = value+MotionManager::GetInstance()->m_Offset[row + 1];
                        GoToCursor(MODVAL_COL, row);
                        printf( "%.4d", Step.position[row + 1] );
                    }
                }
            }
        }
	}
	else if( col == OFFSET_COL )
	{
        MotionManager::GetInstance()->m_Offset[row + 1] = value;
        printf( "%4d ", GetValue() );

        if(InitPose[row + 1] + value >= 0 && InitPose[row + 1] + value <= MX28::MAX_VALUE)
        {
            if(!(Step.position[row + 1] & Action::INVALID_BIT_MASK) && !(Step.position[row + 1] & Action::TORQUE_OFF_BIT_MASK))
            {
                int error;
                if(cm730->WriteWord(row + 1, MX28::P_GOAL_POSITION_L, InitPose[row + 1] + value, &error) == CM730::SUCCESS)
                {
                    if(!(error & CM730::ANGLE_LIMIT))
                    {
                        Step.position[row + 1] = InitPose[row + 1] + value;
                        GoToCursor(MODVAL_COL, row);
                        printf( "%.4d", Step.position[row + 1] );
                    }
                }
            }
        }
        bEdited = true;
	}
    else if( col == MODVAL_COL )
    {
        if(value >= MX28::MIN_VALUE && value <= MX28::MAX_VALUE)
        {
            if(!(Step.position[row + 1] & Action::INVALID_BIT_MASK) && !(Step.position[row + 1] & Action::TORQUE_OFF_BIT_MASK))
            {
                int error;
                if(cm730->WriteWord(row + 1, MX28::P_GOAL_POSITION_L, value, &error) == CM730::SUCCESS)
                {
                    if(!(error & CM730::ANGLE_LIMIT))
                    {
                        Step.position[row + 1] = value;
                        printf( "%.4d", Step.position[row + 1] );
                        MotionManager::GetInstance()->m_Offset[row + 1] = value - InitPose[row + 1];
                        GoToCursor(OFFSET_COL, row);
                        printf( "%4d ", MotionManager::GetInstance()->m_Offset[row + 1] );
                    }
                }
            }
        }
    }
    else if( col == PRSVAL_COL )
    {
        printf( "%.4d", GetValue());
    }
    else if( col == ERRORS_COL )
    {
        printf( "%.4d ", GetValue());
    }
    else if( col == P_GAIN_COL )
    {
        if(value >= 0 && value <= 254)
        {
            if(!(Step.position[row + 1] & Action::INVALID_BIT_MASK) && !(Step.position[row + 1] & Action::TORQUE_OFF_BIT_MASK))
            {
                int error;
                if(cm730->WriteByte(row + 1, MX28::P_P_GAIN, value, &error) == CM730::SUCCESS)
                {
                    P_Gain[row + 1] = value;
                    printf( "%4d", P_Gain[row + 1] );
                }
            }
        }
    }
    else if( col == I_GAIN_COL )
    {
        if(value >= 0 && value <= 254)
        {
            if(!(Step.position[row + 1] & Action::INVALID_BIT_MASK) && !(Step.position[row + 1] & Action::TORQUE_OFF_BIT_MASK))
            {
                int error;
                if(cm730->WriteByte(row + 1, MX28::P_I_GAIN, value, &error) == CM730::SUCCESS)
                {
                    I_Gain[row + 1] = value;
                    printf( "%4d", I_Gain[row + 1] );
                }
            }
        }
    }
    else if( col == D_GAIN_COL )
    {
        if(value >= 0 && value <= 254)
        {
            if(!(Step.position[row + 1] & Action::INVALID_BIT_MASK) && !(Step.position[row + 1] & Action::TORQUE_OFF_BIT_MASK))
            {
                int error;
                if(cm730->WriteByte(row + 1, MX28::P_D_GAIN, value, &error) == CM730::SUCCESS)
                {
                    D_Gain[row + 1] = value;
                    printf( "%4d", D_Gain[row + 1] );
                }
            }
        }
    }

	GoToCursor(col, row);	
}

void ToggleTorque(CM730 *cm730)
{
	if((Col != GOAL_COL && Col != MODVAL_COL) || Row > ID_20_ROW)
		return;

	int id = Row + 1;

	if(Step.position[id] & Action::TORQUE_OFF_BIT_MASK)
	{
		if(cm730->WriteByte(id, MX28::P_TORQUE_ENABLE, 1, 0) != CM730::SUCCESS)
			return;

		int value;
		if(Col == MODVAL_COL)
		{
		    if(cm730->ReadWord(id, MX28::P_GOAL_POSITION_L, &value, 0) != CM730::SUCCESS)
		        return;
		    Step.position[id] = value;
		}
		else if(Col == PRSVAL_COL)
		{
            if(cm730->ReadWord(id, MX28::P_GOAL_POSITION_L, &value, 0) != CM730::SUCCESS)
                return;
            Step.position[id] = value;

		    if(cm730->ReadWord(id, MX28::P_PRESENT_POSITION_L, &value, 0) != CM730::SUCCESS)
		        return;
		    PresentPos.position[id] = value;
		}
		else if(Col == GOAL_COL)
		{
            if(cm730->ReadWord(id, MX28::P_GOAL_POSITION_L, &value, 0) != CM730::SUCCESS)
                return;
            Step.position[id] = value;
            InitPose[id] = value = Step.position[id] - MotionManager::GetInstance()->m_Offset[id];
		}

		//printf("%.4d", value);
		SetValue(cm730, value);
	}
	else
	{
		if(cm730->WriteByte(id, MX28::P_TORQUE_ENABLE, 0, 0) != CM730::SUCCESS)
			return;

		Step.position[id] = Action::TORQUE_OFF_BIT_MASK;
		printf("????");
	}

	GoToCursor(Col, Row);
}

void BeginCommandMode()
{
	Old_Col = Col;
	Old_Row = Row;
	ClearCmd();
	GoToCursor(CMD_COL, CMD_ROW);
	bBeginCommandMode = true;
}

void EndCommandMode()
{
	GoToCursor(Old_Col, Old_Row);
	bBeginCommandMode = false;
}

void HelpCmd()
{
	system("clear");
	printf(" exit               Exits the program.\n");
	printf(" re                 Refreshes the screen.\n");
	printf(" set [value]        Sets value on cursor [value].\n");
    printf(" pgain [value]      Sets ALL actuators' P gain to [value].\n");
    printf(" igain [value]      Sets ALL actuators' I gain to [value].\n");
    printf(" dgain [value]      Sets ALL actuators' D gain to [value].\n");
	printf(" save               Saves offset changes.\n");
	printf(" on/off             Turn On/Off torque from ALL actuators.\n");
	printf(" on/off [index1] [index2] ...  \n"
	       "                    turns On/Off torque from ID[index1] ID[index2]...\n");
	printf("\n");
	printf("       Copyright ROBOTIS CO.,LTD.\n");
	printf("\n");
	printf(" Press any key to continue...");
	_getch();

	DrawPage();
}

void OnOffCmd(CM730 *cm730, bool on, int num_param, int *list)
{
	if(num_param == 0)
	{
		for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
			cm730->WriteByte(id, MX28::P_TORQUE_ENABLE, (int)on, 0);
	}
	else
	{
		for(int i=0; i<num_param; i++)
		{
			if(list[i] >= JointData::ID_R_SHOULDER_PITCH && list[i] <= JointData::ID_HEAD_TILT)
				cm730->WriteByte(list[i], MX28::P_TORQUE_ENABLE, (int)on, 0);
		}
	}

	ReadStep(cm730);
	//DrawStep(7);
	DrawPage();
}


void GainCmd(CM730 *cm730, int value, int pid_col)
{
    if(value < 0 || value > 254)
    {
        PrintCmd("Invalid gain value");
        return;
    }

    int id;
    int n = 0;
    int param[JointData::NUMBER_OF_JOINTS * 2];

    for(id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
    {
        param[n++] = id;
        param[n++] = value;

        if(pid_col == P_GAIN_COL)       P_Gain[id] = value;
        else if(pid_col == I_GAIN_COL)  I_Gain[id] = value;
        else if(pid_col == D_GAIN_COL)  D_Gain[id] = value;
    }

    if(pid_col == P_GAIN_COL)
    {
        cm730->SyncWrite(MX28::P_P_GAIN, 2, JointData::NUMBER_OF_JOINTS - 1, param);
        DrawStep(5);
    }
    else if(pid_col == I_GAIN_COL)
    {
        cm730->SyncWrite(MX28::P_I_GAIN, 2, JointData::NUMBER_OF_JOINTS - 1, param);
        DrawStep(6);
    }
    else if(pid_col == D_GAIN_COL)
    {
        cm730->SyncWrite(MX28::P_D_GAIN, 2, JointData::NUMBER_OF_JOINTS - 1, param);
        DrawStep(7);
    }
}

void SaveCmd(minIni *ini)
{
	if(bEdited == false)
		return;

	MotionManager::GetInstance()->SaveINISettings(ini);
	bEdited = false;
}


