#include <stdio.h>
#include <string.h>
#include "hex2bin.h"


#define MAX_LINE_SIZE	1024
#define ADDRESS_MASK	0x000FFFF0
#define NO_ADDRESS_TYPE_SELECTED	0
#define LINEAR_ADDRESS				1
#define SEGMENTED_ADDRESS			2

bool hex2bin(char *hexFile, unsigned char *pBinBuffer, long *StartAddress, long *BufSize)
{
	FILE *fp;
	unsigned char	Line[MAX_LINE_SIZE];
	unsigned char	Data_Str[MAX_LINE_SIZE];
	unsigned int 	Nb_Bytes;
	unsigned int 	First_Word, Address, Segment, Upper_Address;
	unsigned int    Lowest_Address, Highest_Address, Starting_Address;
	unsigned int 	Phys_Addr, Type;
	unsigned int 	temp;
	int i, iIndex, len;
	bool bRead = true;
	long dwRead;
	
	fp = fopen(hexFile, "r");
	if (!fp)
	{
		printf("Can not find file!\n");
		return false;
	}
	
	Segment = 0x00;
	Upper_Address = 0x00;
	Lowest_Address = MEMORY_MAXSIZE - 1;
	Highest_Address = 0x00;
	Starting_Address = 0x00;
	
	do
	{
		// Read a line from input file.
		for( i=0; i<MAX_LINE_SIZE; i++ )
		{
			dwRead = fread(&Line[i], sizeof(unsigned char), 1, fp);
			if(dwRead <= 0)
			{
				printf("Invalid hex format!\n");
				fclose(fp);
				return false;
			}

			if( Line[i] == 0x0a )
			{
				Line[i] = 0;
				break;
			}
		}

		/* Scan the first two bytes and nb of bytes.
		The two bytes are read in First_Word since it's use depend on the
		record type: if it's an extended address record or a data record.
		*/
		if( Line[0] != ':' )
		{
			printf("Invalid hex format!\n");
			fclose(fp);
			return false;
		}

		len = strlen( (char*)Line );
		for( i=1; i<len; i++ )
		{
			if( Line[i] == ' ' || Line[i] == '	' )
				strcpy( (char*)&Line[i], (char*)&Line[i+1] );
		}

		sscanf( (char*)Line, ":%2x%4x%2x%s", &Nb_Bytes, &First_Word, &Type, Data_Str );

		/* If we're reading the last record, ignore it. */
		switch (Type)
		{
			/* Data record */
			case 0:
				Address = First_Word;
				Phys_Addr = ((Segment << 4) & ADDRESS_MASK) + Address;

				/* Check that the physical address stays in the buffer's range. */
				if ((Phys_Addr + Nb_Bytes) <= MEMORY_MAXSIZE -1)
				{
					/* Set the lowest address as base pointer. */
					if(Lowest_Address > Phys_Addr)
						Lowest_Address = Phys_Addr;
					
					/* Same for the top address. */
					if(Highest_Address < (Phys_Addr + Nb_Bytes - 1))
						Highest_Address = (Phys_Addr + Nb_Bytes - 1);
					
					/* Read the Data bytes. */
					/* Bytes are written in the Memory block even if checksum is wrong. */
					for( i=Nb_Bytes, iIndex=0; i>0; i--, iIndex+=2 )
					{
						sscanf( (char*)&Data_Str[iIndex], "%2x", &temp );
						pBinBuffer[Phys_Addr] = (unsigned char)temp;
						Phys_Addr++;
					}

					/* Read the Checksum value. */
					sscanf( (char*)&Data_Str[iIndex], "%2x", &temp );
				}
				else
				{
					printf("Exceed maximum memory size!\n");
					fclose(fp);
					return false;
				}
				break;

			/* End of file record */
			case 1:
				/* Simply ignore checksum errors in this line. */
				bRead = false;
				break;

			/* Extended segment address record */
			case 2:
				/* First_Word contains the offset. It's supposed to be 0000 so
				we ignore it. */
				sscanf( (char*)Data_Str, "%4x%2x", &Segment, &temp );

				/* Update the current address. */
				Phys_Addr = (Segment << 4) & ADDRESS_MASK;
				break;

			/* Start segment address record */
			case 3:
				/* Nothing to be done since it's for specifying the starting address for
				execution of the binary code */
				break;

			/* Extended linear address record */
			case 4:
				break;

			/* Start linear address record */
			case 5:
				/* Nothing to be done since it's for specifying the starting address for
				execution of the binary code */
				break;

			default:
				printf("Can not support type(%d)\n", Type);
				fclose(fp);
				return false;
		}
		
	} while(bRead == true);

	*BufSize = Highest_Address - Lowest_Address + 1;
	*StartAddress = Lowest_Address;

	fclose(fp);
	return true;
}
