#include "main.h"


// this macro tests for an error and exits the program with a message if there was one
#define EXIT_ON_ERROR(res,comment) if (res != XRV_OK) { printf("Error %d occurred in " comment ": %s\n",res,xsensResultText(res)); exit(1); }

#define CALIB_DATA_OFFSET	3*12 // 3*12 bytes
#define RAWFORCE_OFFSET 16


// used to signal that the user initiated the exit, so we do not wait for an extra keypress
int userQuit = 0;
CmtOutputMode mode;
CmtOutputSettings settings;
unsigned long mtCount = 0;
int screenSensorOffset = 0;
int temperatureOffset = 0;
CmtDeviceId deviceIds[256];
xsens::Cmt3 cmt3;
XsensResultValue res = XRV_OK;
// sample counter
unsigned short sdata = NULL;


int main(void)
{
	//structs to hold data.
	short screenSkipFactor = 10;
	short screenSkipFactorCnt = screenSkipFactor;
	
	

	// Set exit function
	atexit(exitFunc);

	// Perform hardware scan
	doHardwareScan();

	// Give user a (short) chance to see hardware scan results
	Sleep(2000);

	//clear screen present & get the user output mode selection.
	clrscr();
	getUserInputs();

	// Set device to user input settings
	doMtSettings();

	// Wait for first data item(s) to arrive. In production code, you would use a callback function instead (see cmtRegisterCallback function)
	Sleep(20);

	//get the placement offsets, clear the screen and write the fixed headers.
	calcScreenOffset();
	clrscr();
	writeHeaders();
	printf("Unloading...\n");
	UnloadedFS();

	printf("Unloaded voltages Left Back are G0: %.6f, G1: %.6f, G2: %.6f, G3: %.6f, G4: %.6f, G5: %.6f\n", LBUnload[0], LBUnload[1], LBUnload[2], LBUnload[3], LBUnload[4], LBUnload[5]);
	printf("Unloaded voltages Left Front are G0: %.6f, G1: %.6f, G2: %.6f, G3: %.6f, G4: %.6f, G5: %.6f\n", LFUnload[0], LFUnload[1], LFUnload[2], LFUnload[3], LFUnload[4], LFUnload[5]);
	printf("Unloaded voltages Right Back are G0: %.6f, G1: %.6f, G2: %.6f, G3: %.6f, G4: %.6f, G5: %.6f\n", RBUnload[0], RBUnload[1], RBUnload[2], RBUnload[3], RBUnload[4], RBUnload[5]);
	printf("Unloaded voltages Right Front are G0: %.6f, G1: %.6f, G2: %.6f, G3: %.6f, G4: %.6f, G5: %.6f\n", RFUnload[0], RFUnload[1], RFUnload[2], RFUnload[3], RFUnload[4], RFUnload[5]);
	// Initialize packet for data
	Packet* packet = new Packet((unsigned short)mtCount, cmt3.isXm());
	while(!userQuit && res == XRV_OK  /* && sdata < 20 */ )
	{
		
		cmt3.waitForDataMessage(packet);

		//get sample count, goto position & display.
		sdata = packet->getSampleCounter();

		gotoxy(0,0);
		printf("Sample Counter %05hu\n", sdata);
		/*printf("total message size is : %d\n", packet->m_msg.getTotalMessageSize());
		printf("message size without headers is : %d\n", packet->m_msg.getDataSize());
		printf("number of data items in the message: %d\n", packet->m_itemCount);
		printf("individual packet data size (according to outputmode) is : %d\n", packet->getDataSize());
		printf("real individual packet size is : %d\n", packet->m_msg.getDataSize() / packet->m_itemCount);
		printf("sample counter data (2bytes) is at position: %d\n", packet->getInfoList().m_sc);*/

		
		for (int i = 0; i < 8; i++)
		{
			LBraw[i] = shortToVolts(packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 1 * CALIB_DATA_OFFSET + 0 * RAWFORCE_OFFSET + 2 * i));
		}
		/*printf("raw force 1 should be at: %d and is worth G0: %.6f, G1: %.6f, G2: %.6f, G3: %.6f, G4: %.6f, G5: %.6f, G6: %.6f, ref: %.6f\n",
			packet->getInfoList(0).m_calAcc + 1 * CALIB_DATA_OFFSET + 0 * RAWFORCE_OFFSET, LBraw[0], LBraw[1], LBraw[2], LBraw[3], LBraw[4], LBraw[5], LBraw[6], LBraw[7]);*/

		
		for (int i = 0; i < 8; i++)
		{
			LFraw[i] = shortToVolts(packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 2 * CALIB_DATA_OFFSET + 1 * RAWFORCE_OFFSET + 2 * i));
		}
		/*printf("raw force 2 should be at: %d and is worth G0: %.6f, G1: %.6f, G2: %.6f, G3: %.6f, G4: %.6f, G5: %.6f, G6: %.6f, ref: %.6f\n",
			packet->getInfoList(0).m_calAcc + 2 * CALIB_DATA_OFFSET + 1 * RAWFORCE_OFFSET, LFraw[0], LFraw[1], LFraw[2], LFraw[3], LFraw[4], LFraw[5], LFraw[6], LFraw[7]);*/

	
		for (int i = 0; i < 8; i++)
		{
			RBraw[i] = shortToVolts(packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 3 * CALIB_DATA_OFFSET + 2 * RAWFORCE_OFFSET + 2 * i));
		}
		/*printf("raw force 3 should be at: %d and is worth G0: %.6f, G1: %.6f, G2: %.6f, G3: %.6f, G4: %.6f, G5: %.6f, G6: %.6f, ref: %.6f\n",
			packet->getInfoList(0).m_calAcc + 3 * CALIB_DATA_OFFSET + 2 * RAWFORCE_OFFSET, RBraw[0], RBraw[1], RBraw[2], RBraw[3], RBraw[4], RBraw[5], RBraw[6], RBraw[7]);*/

		
		for (int i = 0; i < 8; i++)
		{
			RFraw[i] = shortToVolts(packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 4 * CALIB_DATA_OFFSET + 3 * RAWFORCE_OFFSET + 2 * i));
		}
		/*printf("raw force 4 should be at: %d and is worth G0: %.6f, G1: %.6f, G2: %.6f, G3: %.6f, G4: %.6f, G5: %.6f, G6: %.6f, ref: %.6f\n\n",
			packet->getInfoList(0).m_calAcc + 4 * CALIB_DATA_OFFSET + 3 * RAWFORCE_OFFSET, RFraw[0], RFraw[1], RFraw[2], RFraw[3], RFraw[4], RFraw[5], RFraw[6], RFraw[7]);*/

	
		computeAmpCalMat();		
		computeUDiff();
		computeForceVec();

		
		gotoxy(0, 5 + 0 * screenSensorOffset);
		printf("%.6f\t%.6f\t%.6f", LBforcevec[0], LBforcevec[1], LBforcevec[2]);
		gotoxy(0, 7 + 0 * screenSensorOffset);
		printf("%.6f\t%.6f\t%.6f", LBforcevec[3], LBforcevec[4], LBforcevec[5]);


		gotoxy(0, 5 + 1 * screenSensorOffset);
		printf("%.6f\t%.6f\t%.6f", LFforcevec[0], LFforcevec[1], LFforcevec[2]);
		gotoxy(0, 7 + 1 * screenSensorOffset);
		printf("%.6f\t%.6f\t%.6f", LFforcevec[3], LFforcevec[4], LFforcevec[5]);

		
		gotoxy(0, 5 + 2 * screenSensorOffset);
		printf("%.6f\t%.6f\t%.6f", RBforcevec[0], RBforcevec[1], RBforcevec[2]);
		gotoxy(0, 7 + 2 * screenSensorOffset);
		printf("%.6f\t%.6f\t%.6f", RBforcevec[3], RBforcevec[4], RBforcevec[5]);

		
		gotoxy(0, 5 + 3 * screenSensorOffset);
		printf("%.6f\t%.6f\t%.6f", RFforcevec[0], RFforcevec[1], RFforcevec[2]);
		gotoxy(0, 7 + 3 * screenSensorOffset);
		printf("%.6f\t%.6f\t%.6f", RFforcevec[3], RFforcevec[4], RFforcevec[5]);



		if (screenSkipFactorCnt++ == screenSkipFactor) {
			screenSkipFactorCnt = 0;

			for (unsigned int i = 0; i < mtCount; i++) {	
		
			}	
		}				

		if (_kbhit())
			userQuit = 1;
	}

	delete packet;

	clrscr();
	cmt3.closePort();

	return 0;
}


//////////////////////////////////////////////////////////////////////////////////////////
// Convert the short raw value to the voltage in float.
void UnloadedFS()
{
	Packet* packet = new Packet((unsigned short)mtCount, cmt3.isXm());
	// force unload
	while (!userQuit && res == XRV_OK && sdata < 99)
	{
		cmt3.waitForDataMessage(packet);
		sdata = packet->getSampleCounter();
		
		for (int i = 0; i < 8; i++)
		{
			LBraw[i] = shortToVolts(packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 1 * CALIB_DATA_OFFSET + 0 * RAWFORCE_OFFSET + 2 * i));
			LFraw[i] = shortToVolts(packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 2 * CALIB_DATA_OFFSET + 1 * RAWFORCE_OFFSET + 2 * i));
			RBraw[i] = shortToVolts(packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 3 * CALIB_DATA_OFFSET + 2 * RAWFORCE_OFFSET + 2 * i));
			RFraw[i] = shortToVolts(packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 4 * CALIB_DATA_OFFSET + 3 * RAWFORCE_OFFSET + 2 * i));
		}
		for (int i = 0; i < 6; i++)
		{
			LBUnload[i] += LBraw[i];
			LFUnload[i] += LFraw[i];
			RBUnload[i] += RBraw[i];
			RFUnload[i] += RFraw[i];
		}
		

		if (_kbhit())
			userQuit = 1;
	}

	for (int i = 0; i < 6; i++)
	{
		LBUnload[i] /= 100;
		LFUnload[i] /= 100;
		RBUnload[i] /= 100;
		RFUnload[i] /= 100;
	}
	delete packet;

}

//////////////////////////////////////////////////////////////////////////////////////////
// Convert the short raw value to the voltage in float.
double shortToVolts(const uint16_t raw)
{
	double U = double(raw);
	U *= 4.999924 / 65535;
	return U;
}


//////////////////////////////////////////////////////////////////////////////////////////
// compute amplified calibration matrixes (raw/amplifier gain/excitation)
void computeAmpCalMat()
{
	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 6; j++)
		{
			ampCalMatLB[i][j] = rawCalMatLB[i][j] / ampGain / LBraw[6];
			ampCalMatLF[i][j] = rawCalMatLF[i][j] / ampGain / LFraw[6];
			ampCalMatRB[i][j] = rawCalMatRB[i][j] / ampGain / RBraw[6];
			ampCalMatRF[i][j] = rawCalMatRF[i][j] / ampGain / RFraw[6];
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////////////
// compute difference between current voltages and unloaded voltages
void computeUDiff()
{
	for (int i = 0; i < 6; i++)
	{
		LBdiff[i] = LBraw[i] - LBUnload[i];
		LFdiff[i] = LFraw[i] - LFUnload[i];
		RBdiff[i] = RBraw[i] - RBUnload[i];
		RFdiff[i] = RFraw[i] - RFUnload[i];
	}
}

//////////////////////////////////////////////////////////////////////////////////////////
// process FT vectors from voltages (amplified calibration matrixes * diff voltage vectors
void computeForceVec(void)
{
	// Reseting values
	for (int i = 0; i < 6; i++)
	{
		LBforcevec[i] = 0.;
		LFforcevec[i] = 0.;
		RBforcevec[i] = 0.;
		RFforcevec[i] = 0.;
	}

	for (int i = 0; i < 6; i++) // i: rows of cal mat
	{
		for (int j = 0; j < 6; j++) // j: column of voltage vect
		{
				LBforcevec[i] += ampCalMatLB[i][j] * LBdiff[j];
				LFforcevec[i] += ampCalMatLF[i][j] * LFdiff[j];
				RBforcevec[i] += ampCalMatRB[i][j] * RBdiff[j];
				RFforcevec[i] += ampCalMatRF[i][j] * RFdiff[j];
		}
	}
}

//////////////////////////////////////////////////////////////////////////
// doHardwareScan
//
// Checks available COM ports and scans for MotionTrackers
void doHardwareScan()
{
	XsensResultValue res;
	List<CmtPortInfo> portInfo;
	unsigned long portCount = 0;

	printf("Scanning for connected Xsens devices...");
	xsens::cmtScanPorts(portInfo);
	portCount = portInfo.length();
	printf("done\n");

	if (portCount == 0) {
		printf("No Xsens devices found\n\n");
		exit(0);
	}

	for(int i = 0; i < (int)portCount; i++) {	
		printf("Using COM port %d at %d baud\n\n",
			(long) portInfo[i].m_portNr, portInfo[i].m_baudrate);	
	}

	printf("Opening ports...");
	//open the port which the device is connected to and connect at the device's baudrate.
	for(int p = 0; p < (int)portCount; p++){
		res = cmt3.openPort(portInfo[p].m_portNr, portInfo[p].m_baudrate);
		EXIT_ON_ERROR(res,"cmtOpenPort");  
	}
	printf("done\n\n");

	//get the Mt sensor count.
	printf("Retrieving MotionTracker count (excluding attached Xbus Master(s))\n");
	mtCount = cmt3.getMtCount();
	mtCount = mtCount;
	printf("MotionTracker count: %i\n\n",mtCount);

	// retrieve the device IDs 
	printf("Retrieving MotionTrackers device ID(s)\n");
	for(unsigned int j = 0; j < mtCount; j++ ){
		res = cmt3.getDeviceId((unsigned char)(j+1), deviceIds[j]);
		EXIT_ON_ERROR(res,"getDeviceId");
		printf("Device ID at busId %i: %08x\n",j+1,(long) deviceIds[j]);
	}	
}

//////////////////////////////////////////////////////////////////////////
// getUserInputs
//
// Request user for output data
void getUserInputs()
{
	mode = 7;
	//do{
	//	printf("Select operation:\n");
	//	printf("1 - Force shoes unloaded \n");
	//	printf("2 - Measure and print data\n");
	//	printf("3 - Measure, print and send data\n");
	//	printf("Enter your choice: ");
	//	scanf_s("%d", &mode);
	//	// flush stdin
	//	while (getchar() != '\n') continue;

	//	if (mode < 1 || mode > 3) {
	//		printf("\n\nPlease enter a valid output mode\n");
	//	}
	//}while(mode < 1 || mode > 6);
	//clrscr();

	switch(mode)
	{
	case 1:
		mode = CMT_OUTPUTMODE_CALIB;
		break;
	case 2:
		mode = CMT_OUTPUTMODE_ORIENT;
		break;
	case 3:
		mode = CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_ORIENT;
		break;
	case 4:
		mode = CMT_OUTPUTMODE_TEMP | CMT_OUTPUTMODE_CALIB;
		break;
	case 5:
		mode = CMT_OUTPUTMODE_TEMP | CMT_OUTPUTMODE_ORIENT;
		break;
	case 6:
		mode = CMT_OUTPUTMODE_TEMP | CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_ORIENT;
		break;
	case 7:
		mode = CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_AUXILIARY /* | CMT_OUTPUTMODE_CALIB*/;
		break;
	}

	if ((mode & CMT_OUTPUTMODE_AUXILIARY) != 0)
	{
		settings = CMT_OUTPUTSETTINGS_AUXILIARYMODE_FORCE;
		settings = 0x00001c00;
		//settings = CMT_OUTPUTSETTINGS_AUXILIARYMODE_MASK;
		//settings = 0x00000000;
		printf("Auxiliary mode, sending force info \n", settings);
	}
	else{
		settings = 0;
	}
	settings |= CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;

}

//////////////////////////////////////////////////////////////////////////
// doMTSettings
//
// Set user settings in MTi/MTx
// Assumes initialized cmt3 class with open COM port
void doMtSettings(void) 
{
	XsensResultValue res;

	//res = cmt3.getProcessingFlags();

	// set sensor to config sate
	res = cmt3.gotoConfig();
	EXIT_ON_ERROR(res,"gotoConfig");

	unsigned short sampleFreq;
	sampleFreq = cmt3.getSampleFrequency();

	// set the device output mode for the device(s)
	printf("Configuring your mode selection\n");
	CmtDeviceMode deviceMode(mode, settings, sampleFreq);
	for(unsigned int i = 0; i < mtCount; i++){
		res = cmt3.setDeviceMode(deviceMode,true, deviceIds[i]);
		EXIT_ON_ERROR(res,"setDeviceMode");
	}

	// start receiving data
	res = cmt3.gotoMeasurement();
	EXIT_ON_ERROR(res,"gotoMeasurement");
}

//////////////////////////////////////////////////////////////////////////
// writeHeaders
//
// Write appropriate headers to screen
void writeHeaders()
{
	for (unsigned int i = 0; i < mtCount; i++) {	
		gotoxy(0, 2 + i * screenSensorOffset);
		printf("MotionTracker %d\n", i + 1);

		gotoxy(0,3 + i * screenSensorOffset);
		printf("Calibrated sensor data");
		gotoxy(0,4 + i * screenSensorOffset);
		printf(" Fx\t \t Fy\t \t Fz");
		gotoxy(43, 5 + i * screenSensorOffset);
		printf("Newtons");
		gotoxy(0,6 + i * screenSensorOffset);
		printf(" Tx\t \t Ty\t \t Tz");
		gotoxy(43, 7 + i * screenSensorOffset);
		printf("N.m");
		gotoxy(0, 8 + i * screenSensorOffset);
		/*printf("Udiff is:");
		gotoxy(0, 9 + i * screenSensorOffset);*/
			
		
		

	}
}

//////////////////////////////////////////////////////////////////////////
// calcScreenOffset
//
// Calculates offset for screen data with multiple sensors.
void calcScreenOffset()
{
	// 1 line for "Sensor ..."
	screenSensorOffset += 1;
	if ((mode & CMT_OUTPUTMODE_CALIB) != 0)
		screenSensorOffset += 6;
	if ((mode & CMT_OUTPUTMODE_ORIENT) != 0) {
		switch(settings & CMT_OUTPUTSETTINGS_ORIENTMODE_MASK) {
		case CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION:
			screenSensorOffset += 4;
			break;
		case CMT_OUTPUTSETTINGS_ORIENTMODE_EULER:
			screenSensorOffset += 4;
			break;
		case CMT_OUTPUTSETTINGS_ORIENTMODE_MATRIX:
			screenSensorOffset += 6;
			break;
		default:
			;
		}
	}
}

//////////////////////////////////////////////////////////////////////////
// clrscr
//
// Clear console screen
void clrscr() 
{
#ifdef WIN32
	CONSOLE_SCREEN_BUFFER_INFO csbi;
	HANDLE hStdOut = GetStdHandle(STD_OUTPUT_HANDLE);
	COORD coord = {0, 0};
	DWORD count;

	GetConsoleScreenBufferInfo(hStdOut, &csbi);
	FillConsoleOutputCharacter(hStdOut, ' ', csbi.dwSize.X * csbi.dwSize.Y, coord, &count);
	SetConsoleCursorPosition(hStdOut, coord);
#else
	int i;

	for (i = 0; i < 100; i++)
		// Insert new lines to create a blank screen
		putchar('\n');
	gotoxy(0,0);
#endif
}

//////////////////////////////////////////////////////////////////////////
// gotoxy
//
// Sets the cursor position at the specified console position
//
// Input
//	 x	: New horizontal cursor position
//   y	: New vertical cursor position
void gotoxy(int x, int y)
{
#ifdef WIN32
	COORD coord;
	coord.X = x;
	coord.Y = y;
	SetConsoleCursorPosition(GetStdHandle(STD_OUTPUT_HANDLE), coord);
#else
	char essq[100];		// String variable to hold the escape sequence
	char xstr[100];		// Strings to hold the x and y coordinates
	char ystr[100];		// Escape sequences must be built with characters

	/*
	** Convert the screen coordinates to strings
	*/
	sprintf(xstr, "%d", x);
	sprintf(ystr, "%d", y);

	/*
	** Build the escape sequence (vertical move)
	*/
	essq[0] = '\0';
	strcat(essq, "\033[");
	strcat(essq, ystr);

	/*
	** Described in man terminfo as vpa=\E[%p1%dd
	** Vertical position absolute
	*/
	strcat(essq, "d");

	/*
	** Horizontal move
	** Horizontal position absolute
	*/
	strcat(essq, "\033[");
	strcat(essq, xstr);
	// Described in man terminfo as hpa=\E[%p1%dG
	strcat(essq, "G");

	/*
	** Execute the escape sequence
	** This will move the cursor to x, y
	*/
	printf("%s", essq);
#endif
}

//////////////////////////////////////////////////////////////////////////
// exitFunc
//
// Closes cmt nicely
void exitFunc(void)
{
	// Close any open COM ports
	cmt3.closePort();
	
	// get rid of keystrokes before we post our message
	while (_kbhit()) _getch();

	// wait for a keypress
	if (!userQuit)
	{
		printf("Press a key to exit\n");
		_getch();
	}
}
