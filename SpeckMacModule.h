/*!
	\file SpeckMacModule.h
	\author Siddhu Warrier, University of Edinburgh
	\brief Defintions and defines for the SpeckMAC-D algorithm.
*/

#ifndef SPECKMACMODULE
#define SPECKMACMODULE

#include <vector>
#include <omnetpp.h>
//#include "App_GenericDataPacket_m.h"
#include "App_ControlMessage_m.h"
#include "NetworkGenericFrame_m.h"
#include "NetworkControlMessage_m.h"
#include "MacGenericFrame_m.h"
#include "MacControlMessage_m.h"
#include "RadioControlMessage_m.h"
#include "ResourceGenericManager.h"
#include "RadioModule.h"
#include "DebugInfoWriter.h"
using namespace std;

#define TRUE 1 

#define FALSE 0

#define MAC_BUFFER_ARRAY_SIZE macBufferSize+1 //!< \def Define size of the transmit buffer.

#define BUFFER_IS_EMPTY  (headTxBuffer==tailTxBuffer) //!< \def Macro to check if buffer is empty.

#define BUFFER_IS_FULL  (getTXBufferSize() >= macBufferSize) //!< \def Macro to check if buffer is full.

#define CARRIER_SENSE_INTERVAL 0.0001 //!< \def Interval for which radio performs Carrier Sense.

#define DRIFTED_TIME(time) ((time) * cpuClockDrift)

#define EV   ev.disabled() ? (ostream&)ev : ev //!< \def Output to Primary-Output.txt

#define SLEEP 0 

#define WAKEUP 1

#define CASTALIA_DEBUG (!printDebugInfo)?(ostream&)DebugInfoWriter::getStream():DebugInfoWriter::getStream() //!< \def Macro to print debug information.

// #define STATISTICS //!< \def Enable statistics calculation.

/*!
	\enum
	\brief Defines the list of states that the SpeckMAC-D algorithm can take.
*/
enum MacStates
{
	MAC_STATE_DEFAULT = 2204,
	MAC_STATE_TX = 2205,
	MAC_STATE_CARRIER_SENSING = 2206,
	MAC_STATE_EXPECTING_RX = 2207, // because of the duty cycle there is a mode when we are expecting a train of beacons and data
	MAC_STATE_TRY_TX = 2208
};

/*!
 \class SpeckMacModule
 \author Siddhu Warrier, University of Edinburgh
 \brief The SpeckMAC algorithm class. 

 This class defines the class members and member functions used in the SpeckMAC algorithm. 
The class members are of two types:
1. Class members defined in the .ned file.
2. Custom class members.

\note Some member functions are declared inline, because they are called only once in the code.	
*/
class SpeckMacModule : public cSimpleModule
{
	private:
		// NED file parameters
		
		bool printDebugInfo;  //!< Indicate whether debug information must be printed out.
		bool printStateTransitions; //!< Indicate whether state transitions should be printed out.
		
		double sleepInterval; //!< interval for which the radio is put to sleep.
		double listenInterval; //!< interval for which the radio is turned on.
		double randomTxOffset; //!< random offset to get nodes out of sync. \bug Not entirely effective.
		
		int maxMacFrameSize; //!< Maximum MAC frame size.
		int macBufferSize; //!< the size of the transmission Buffer.
		int macFrameOverhead; //!< the size of the MAC headers.
		
		//! Custom Class parameters
		RadioModule *radioModule;	//!< a pointer to the object of the Radio Module (used for direct method calls).
		ResourceGenericManager *resMgrModule;	//!< a pointer to the object of the Radio Module (used for direct method calls).
		MAC_GenericFrame **schedTXBuffer;		//!< a circular buffer that holds frames for transmission.
		
		cMessage *dutyCycleSleepMsg;	//!< Duty cycle sleep message.
		cMessage *dutyCycleWakeupMsg;	//!< Duty cycle wakeup message.
		MAC_ControlMessage *selfExitCSMsg; //!< message to exit Carrier sense; indicating channel is free.
		
		bool doTx; //!< Indicate if a transmission has to be performed.
		
		int self; //!< The node's ID.
		int macState; //!< The state of the MAC layer.
		int headTxBuffer; //!< The position of the head of the buffer (pop position).
		int tailTxBuffer; //!< The position of the tail of the buffer (pop position).
		int disabled; //!< bool, to indicate if MAC module is operational or not.
		int phyLayerOverhead; //!< The physical layer overhead.
		int redundancy; //!< The number of redundant retransmissions for SpeckMAC-D

#ifdef STATISTICS		
		int numRecd; //!< statistics member to count number of received packets.
		int numSent; //!< statistics member to count number of sent packets.
#endif

		double epsilon;
		double cpuClockDrift; //!< Clock drift of CPU.
		double radioDataRate; //!< Data rate of radio (e.g. 250 kbps for CC2420)
		double radioDelayForValidCS; //!< Time required before radio can perform Carrier Sense.
		double dataTXtime; //!< Time to transmit the MAC Frame.
		double lastWakeupTime; //!< Time last wakeup message was received.
		
	protected:
		virtual void initialize();
		virtual void finish();
		virtual void handleMessage(cMessage *msg);
		
		void readIniFileParameters();
		void setRadioState(MAC_ContorlMessageType typeID, double delay = 0.0);
		inline void dutyCycle (int typeID);
		inline void handleNetworkLayerFrame(cMessage *msg);
		void pushFrameIntoBuffer(cMessage *msg);
		int pushBuffer(MAC_GenericFrame *theFrame);
		void initiateCarrierSense();
		void performCarrierSense();
		int encapsulateNetworkFrame(Network_GenericFrame *networkFrame, MAC_GenericFrame *retFrame);
		int getTXBufferSize(void);
		int resolvDestination(const char *);
		void carrierFree();
		void carrierBusy();
		inline void sendData();
		inline void finishDataTransmission();
		MAC_GenericFrame *popTxBuffer();
};

#endif
