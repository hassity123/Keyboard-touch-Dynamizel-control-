// Each Dynamixel is running within a switch-case statement

// This code is designed for using a Dynamixel xm430-340r, and a USB2DYNAMIXEL.

// To use another Dynamixel model,we have to follow the E-Manual(support.robotis.com)

// Using the R+ manager of dynamixel we can change dynamixel values with in the company specified range

/*This source includes below to get key input interruption while the example is running. Actual functions for getting the input is described in a little below. */
#ifdef __linux__
#include <unistd.h>
#include <fcntl.h>
#include <temios.h>
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif
#include <stdlib.h> // The function  abs()  is in the example code, and it needs  stdlib.h  to be included.
#include <stdio.h> // The example shows Dynamixel status in sequence by the function  printf() . So here  stdio.h  is needed 
#include "dynamixel_sdk.h"        //   All libraries of Dynamixel SDK are linked with the header file  dynamixel_sdk.h .




// Control table address  Dynamixel series have their own control tables: Addresses and Byte Length in each items. To control one of the items, 
// its address (and length if necessary) is required. Find your requirements in http://support.robotis.com/

#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model

#define ADDR_PRO_GOAL_POSITION          116

#define ADDR_PRO_PRESENT_POSITION       132




// Protocol version : Dynamixel uses either or both protocols: Protocol 1.0 and Protocol 2.0. Choose one of the Protocol which is appropriate in the Dynamixel

#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting

#define DXL_ID                          1                   // Dynamixel ID: 1

#define DXL2_ID                         2                   // Dynamixel ID: 2

#define DXL3_ID                         3                   // Dynamixel ID: 3

#define DXL4_ID                         4                   // Dynamixel ID: 4

#define DXL5_ID                         5                   // Dynamixel ID: 4

#define BAUDRATE                        57600

#define DEVICENAME                      "COM3"      // Check which port is being used on your controller





#define TORQUE_ENABLE                   1                   // Value for enabling the torque

#define TORQUE_DISABLE                  0                   // Value for disabling the torque

#define DXL_MINIMUM_POSITION_VALUE      0                   // Dynamixel will rotate between this value

#define DXL_2_MINIMUM_POSITION_VALUE    0                   //  This can be set from zero to any value to set the minimum angular position of a motor 

#define DXL_3_MINIMUM_POSITION_VALUE    0  

#define DXL_4_MINIMUM_POSITION_VALUE    0  

#define DXL_5_MINIMUM_POSITION_VALUE    0  

#define DXL_MAXIMUM_POSITION_VALUE      4095                // This value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)

#define DXL_2_MAXIMUM_POSITION_VALUE    4095		

#define DXL_3_MAXIMUM_POSITION_VALUE    4095

#define DXL_4_MAXIMUM_POSITION_VALUE    4095

#define DXL_5_MAXIMUM_POSITION_VALUE    4095

#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold




#define ESC_ASCII_VALUE                 0x1b          // escape key to interrupt 

#define key_1                           0x31          // number one  key to run motor #1 

#define key_2                           0x32          // number two key to run motor  #2 

#define key_3                           0x33          //  number three key to run  motor #3 

#define key_4                           0x34          // number four key to run motor #4 

#define key_5                           0x35          // number four key to run motor #5




int getch()
{
#ifdef __linux__
	struct termios oldt, newt;
	int ch;
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	ch = getchar();
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	return ch;
#elif defined(_WIN32) || defined(_WIN64)
	return _getch();
#endif
}




int kbhit(void)

{

#ifdef __linux__

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
	if (ch != EOF)
	{
		ungetc(ch, stdin);
		return 1;
	}
	return 0;
#elif defined(_WIN32) || defined(_WIN64)
	return _kbhit();
#endif
}




int main()
{
	// Initialize PortHandler instance
	// Set the port path
	// Get methods and members of PortHandlerLinux or PortHandlerWindows
	dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

	// Initialize PacketHandler instance

	// Set the protocol version

	// Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler

	dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

	int index = 0;

	int dxl_comm_result = COMM_TX_FAIL;             // Communication result

	int dxl_goal_position[2] = { DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE };          // Goal position of motor #1

	int dxl_2_goal_position[2] = { DXL_2_MINIMUM_POSITION_VALUE, DXL_2_MAXIMUM_POSITION_VALUE };    // Goal position of motor #2

	int dxl_3_goal_position[2] = { DXL_3_MINIMUM_POSITION_VALUE, DXL_3_MAXIMUM_POSITION_VALUE };    // Goal position of motor #3

	int dxl_4_goal_position[2] = { DXL_4_MINIMUM_POSITION_VALUE, DXL_4_MAXIMUM_POSITION_VALUE };     // Goal position of motor #4

	int dxl_5_goal_position[2] = { DXL_4_MINIMUM_POSITION_VALUE, DXL_4_MAXIMUM_POSITION_VALUE };     // Goal position of motor #5


	uint8_t dxl_error = 0;                          // Dynamixel error

	int32_t dxl_present_position = 0;               // Present position  motor #1

	int32_t dxl_2_present_position = 0;             // Present position  motor #2

	int32_t dxl_3_present_position = 0;             // Present position  motor #3

	int32_t dxl_4_present_position = 0;             // Present position  motor #4

	int32_t dxl_5_present_position = 0;             // Present position  motor #5


	// Open port

	if (portHandler->openPort())
	{
		printf("Succeeded to open the port!\n");
	}
	else
	{
		printf("Failed to open the port!\n");
		printf("Press any key to terminate...\n");
		getch();
		return 0;
	}
	// Set port baudrate
	if (portHandler->setBaudRate(BAUDRATE))
	{
		printf("Succeeded to change the baudrate!\n");
	}
	else
	{
		printf("Failed to change the baudrate!\n");
		printf("Press any key to terminate...\n");
		getch();
		return 0;
	}




	// Enable Dynamixel Torque #1 

	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

	if (dxl_comm_result != COMM_SUCCESS)
	{
		packetHandler->printTxRxResult(dxl_comm_result);
	}
	else if (dxl_error != 0)
	{
		packetHandler->printRxPacketError(dxl_error);
	}
	else
	{
		printf("Dynamixel #1 has been successfully connected ! \n");
	}


	// Enable Dynamixel #2 Torque

	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

	if (dxl_comm_result != COMM_SUCCESS)
	{
		packetHandler->printTxRxResult(dxl_comm_result);
	}
	else if (dxl_error != 0)
	{
		packetHandler->printRxPacketError(dxl_error);
	}

	else
	{
		printf("Dynamixel #2 has been successfully connected! \n");
	}
	// Enable Dynamixel #3 Torque

	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

	if (dxl_comm_result != COMM_SUCCESS)
	{
		packetHandler->printTxRxResult(dxl_comm_result);
	}
	else if (dxl_error != 0)
	{
		packetHandler->printRxPacketError(dxl_error);
	}
	else
	{
		printf("Dynamixel #3 has been successfully connected ! \n");
	}

	// Enable Dynamixel #4 Torque

	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

	if (dxl_comm_result != COMM_SUCCESS)
	{
		packetHandler->printTxRxResult(dxl_comm_result);
	}
	else if (dxl_error != 0)
	{
		packetHandler->printRxPacketError(dxl_error);
	}
	else
	{
		printf("Dynamixel #4 has been successfully connected ! \n");
	}

	// Enable Dynamixel #5 Torque

	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

	if (dxl_comm_result != COMM_SUCCESS)
	{
		packetHandler->printTxRxResult(dxl_comm_result);
	}
	else if (dxl_error != 0)
	{
		packetHandler->printRxPacketError(dxl_error);
	}
	else
	{
		printf("Dynamixel #5 has been successfully connected ! \n");
	}

	while (1)        
	{
	
		char pressed_key; 
		printf("Press key to continue! (or press ESC to quit!) \n");

		pressed_key = getch();

		if (pressed_key == ESC_ASCII_VALUE)
			break;
		switch (pressed_key) {
		case key_1:
			dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_POSITION, dxl_goal_position[index], &dxl_error); // write goal position of motor #1 
			if (dxl_comm_result != COMM_SUCCESS)
			{
				packetHandler->printTxRxResult(dxl_comm_result);
			}
			else if (dxl_error != 0)
			{
				packetHandler->printRxPacketError(dxl_error);
			}
			do
			{
				// Read present position motor #1
				dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error); 

				if (dxl_comm_result != COMM_SUCCESS)
				{
					packetHandler->printTxRxResult(dxl_comm_result);
				}
				else if (dxl_error != 0)
				{
					packetHandler->printRxPacketError(dxl_error);
				}
				printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_ID, dxl_goal_position[index], dxl_present_position);
			} while (abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD);
			break;

		case key_2:
			dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_POSITION, dxl_2_goal_position[index], &dxl_error); // write goal position of motor #2

			if (dxl_comm_result != COMM_SUCCESS)

			{

				packetHandler->printTxRxResult(dxl_comm_result);

			}

			else if (dxl_error != 0)

			{

				packetHandler->printRxPacketError(dxl_error);
			}
			do
			{
				// Read present postion of motor #2
				dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_2_present_position, &dxl_error);

				if (dxl_comm_result != COMM_SUCCESS)

				{

					packetHandler->printTxRxResult(dxl_comm_result);

				}

				else if (dxl_error != 0)

				{

					packetHandler->printRxPacketError(dxl_error);

				}

				printf("[ID:%03d] GoalPos:%03d  PresPos:%03d \n", DXL2_ID, dxl_2_goal_position[index], dxl_2_present_position);



			} while (abs(dxl_2_goal_position[index] - dxl_2_present_position) > DXL_MOVING_STATUS_THRESHOLD);

			break;

		case key_3:

			dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_GOAL_POSITION, dxl_3_goal_position[index], &dxl_error); // write goal position of motor #3
			if (dxl_comm_result != COMM_SUCCESS)
			{
				packetHandler->printTxRxResult(dxl_comm_result);
			}
			else if (dxl_error != 0)
			{
				packetHandler->printRxPacketError(dxl_error);
			}
			do
			{

				// Read present position motor #3

				dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_3_present_position, &dxl_error);

				if (dxl_comm_result != COMM_SUCCESS)

				{

					packetHandler->printTxRxResult(dxl_comm_result);

				}

				else if (dxl_error != 0)

				{

					packetHandler->printRxPacketError(dxl_error);

				}



				printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL3_ID, dxl_3_goal_position[index], dxl_3_present_position);

			} while (abs(dxl_3_goal_position[index] - dxl_3_present_position) > DXL_MOVING_STATUS_THRESHOLD);

			break;
		
		case key_4:

			dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_GOAL_POSITION, dxl_4_goal_position[index], &dxl_error); // write goal position of motor #4
			if (dxl_comm_result != COMM_SUCCESS)
			{
				packetHandler->printTxRxResult(dxl_comm_result);
			}
			else if (dxl_error != 0)
			{
				packetHandler->printRxPacketError(dxl_error);
			}
			do
			{
				// Read present position motor #4

				dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_4_present_position, &dxl_error);

				if (dxl_comm_result != COMM_SUCCESS)
				{
					packetHandler->printTxRxResult(dxl_comm_result);
				}
				else if (dxl_error != 0)
				{
					packetHandler->printRxPacketError(dxl_error);
				}



				printf("[ID:%03d] GoalPos:%03d  PresPos:%03d \n", DXL4_ID, dxl_4_goal_position[index], dxl_4_present_position);

			} while (abs(dxl_4_goal_position[index] - dxl_4_present_position) > DXL_MOVING_STATUS_THRESHOLD);
			break;
		case key_5:
			dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_GOAL_POSITION, dxl_5_goal_position[index], &dxl_error); // write goal position of motor #5 
			if (dxl_comm_result != COMM_SUCCESS)
			{
				packetHandler->printTxRxResult(dxl_comm_result);
			}
			else if (dxl_error != 0)
			{
				packetHandler->printRxPacketError(dxl_error);
			}
			do
			{
				// Read present position motor #1
				dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_5_present_position, &dxl_error);

				if (dxl_comm_result != COMM_SUCCESS)
				{
					packetHandler->printTxRxResult(dxl_comm_result);
				}
				else if (dxl_error != 0)
				{
					packetHandler->printRxPacketError(dxl_error);
				}
				printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL5_ID, dxl_5_goal_position[index], dxl_5_present_position);
			} while (abs(dxl_5_goal_position[index] - dxl_5_present_position) > DXL_MOVING_STATUS_THRESHOLD);
			break;
			// ~~~~~~~~~~~~~~~~~~~~~~~
		default:
			break;
		}
		// Change goal position
		if (index == 0)
		{
			index = 1;
		}
		else
		{
			index = 0;
		}
	}
	// Disable Dynamixel 1 Torque

	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

	if (dxl_comm_result != COMM_SUCCESS)
	{
		packetHandler->printTxRxResult(dxl_comm_result);
	}
	else if (dxl_error != 0)
	{
		packetHandler->printRxPacketError(dxl_error);
	}

	// Disable Dynamixel 2 Torque

	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

	if (dxl_comm_result != COMM_SUCCESS)
	{
		packetHandler->printTxRxResult(dxl_comm_result);
	}
	else if (dxl_error != 0)
	{
		packetHandler->printRxPacketError(dxl_error);
	}

	// Disable Dynamixel 3 Torque

	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

	if (dxl_comm_result != COMM_SUCCESS)
	{
		packetHandler->printTxRxResult(dxl_comm_result);
	}
	else if (dxl_error != 0)
	{
		packetHandler->printRxPacketError(dxl_error);
	}
	// Disable Dynamixel 4 Torque

	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

	if (dxl_comm_result != COMM_SUCCESS)
	{
		packetHandler->printTxRxResult(dxl_comm_result);
	}
	else if (dxl_error != 0)
	{
		packetHandler->printRxPacketError(dxl_error);
	}

	// Disable Dynamixel 5 Torque

	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

	if (dxl_comm_result != COMM_SUCCESS)
	{
		packetHandler->printTxRxResult(dxl_comm_result);
	}
	else if (dxl_error != 0)
	{
		packetHandler->printRxPacketError(dxl_error);
	}

	// Close port
	portHandler->closePort();
	return 0;
}