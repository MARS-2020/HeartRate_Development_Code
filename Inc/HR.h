//Heart Rate Arrays for Initialization and Reading

//Table 8:

//Start Algorithm
#define arr_1_2[3]		{0x10, 0x00, 0x03}	//set output mode to sensor
#define arr_1_3[3]		{0x10, 0x01, 0x0F}	//set sensor hub interrupt threshold
#define arr_1_4[3]		{0x52, 0x00, 0x01}  //enable AGC
#define arr_1_6[3]		{0x44, 0x03, 0x01}  //03 = AFE (30101 vs 30102?)
#define arr_1_7[3]		{0x52, 0x02, 0x01}  //Enable HR/SpO2 algorithm

//Read Sample
#define arr_2_1[2]		{0x00, 0x00}		//Read sensor hub status byte
#define arr_2_2[2]		{0x12, 0x00}		//get number of samples in FIFO
#define arr_2_3[2]		{0x12, 0x01}		//read data stored in FIFO

//Host End Procedure
#define arr_3_1[3]		{0x44, 0x03, 0x00}	//Disable AFE
#define arr_3_3[3]		{0x52, 0x02, 0x00}	//disable algorithm
