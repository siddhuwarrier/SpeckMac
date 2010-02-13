/*!
	\file SpeckMacModule.cc
	\author Siddhu Warrier, University of Edinburgh
	\brief Implements the SpeckMAC-D algorithm.
	Extensively reuses code from the TunableMAC implementation written by Athanassios Boulis and Dimosthenis Pediaditakis, NICTA.
*/

#include "SpeckMacModule.h"

#define BLOCKING
#define DEBUG

Define_Module(SpeckMacModule);

/*!
	\brief Initialises the SpeckMAC module.
	
	This method is called when the simulation starts. It loads all the parameters, obtains references to objects of the radio module and the resource manager, determines the CPU clock drift (in order to avoid the nodes remaining synchronised to each other), and disables the module. The MAC algorithm thus starts executing only when a message is received from a higher layer (namely, the network layer).
*/
void SpeckMacModule::initialize()
{
	readIniFileParameters();

	self = parentModule()->parentModule()->index();

	// get a valid reference to the object of the Radio module so that we can make direct calls to its public methods
	// instead of using extra messages & message types for tighlty couplped operations.
	radioModule = check_and_cast<RadioModule*>(gate("toRadioModule")->toGate()->ownerModule());
	radioDataRate = (double) radioModule->par("dataRate");
	radioDelayForValidCS = ((double) radioModule->par("delayCSValid"))/1000.0; // parameter given in ms in the omnetpp.ini

	phyLayerOverhead = radioModule->par("phyFrameOverhead"); // get the physical layer overhead
	
	// get a valid reference to the object of the Resources Manager module so that we can make direct calls to its public methods
	// instead of using extra messages & message types for tighlty couplped operations.
	cModule *parentParent = parentModule()->parentModule();
	if(parentParent->findSubmodule("nodeResourceMgr") != -1)
	{
		resMgrModule = check_and_cast<ResourceGenericManager*>(parentParent->submodule("nodeResourceMgr"));
	}
	else
	{
		opp_error("\n[Mac]:\n Error in geting a valid reference to  nodeResourceMgr for direct method calls.");
	}
	
	cpuClockDrift = resMgrModule->getCPUClockDrift();
	
	// Set radio to sleep
	setRadioState(MAC_2_RADIO_ENTER_SLEEP);
	
	macState = MAC_STATE_DEFAULT;
	
	schedTXBuffer = new MAC_GenericFrame*[MAC_BUFFER_ARRAY_SIZE];
	
	headTxBuffer = 0;
	tailTxBuffer = 0;
	
	// Statistics
#ifdef STATISTICS
	numRecd = 0;
	numSent = 0;
#endif
	// maxSchedTXBufferSizeRecorded = 0;

	epsilon = 0.000001f;

	disabled = TRUE;
	
	dutyCycleSleepMsg = NULL;
	dutyCycleWakeupMsg = NULL;
	
	CASTALIA_DEBUG << "\nSpeckMAC_"<<self<<"[t = "<< simTime() << "]: Initialization complete";
	
	doTx = FALSE;
}

/*!
	\brief Clean-up method executed before the simulation stops.
	
	This method is called when the simulation stops executing. It clears the transmission buffer, deallocates memory, and (optional) prints statistics collected by the MAC layer. To disable the last, comment the #define STATISTICS line.
 */
void SpeckMacModule::finish()
{
	MAC_GenericFrame *macMsg;
	
	while(!BUFFER_IS_EMPTY)
	{
		macMsg = popTxBuffer();

		cancelAndDelete(macMsg);

		macMsg = NULL;
	}
	
#ifdef STATISTICS
	EV << numRecd << "," << numSent << "\t";
#endif
}

/*!
	\fn SpeckMacModule::handleMessage(cMessage *)
	\brief Handles messages received by the MAC module.
	
	This method is executed whenever the MAC module receives a message. The message may be received from itself, the network module, or the radio module.

	The received message is passed to this method as an argument.
	
	The method handles the following kinds of messages:
	\par (a) APP_NODE_STARTUP: 
	This message is received from the network module to start the MAC module up. When this message is received, the module is enabled, and radio duty cycling is initialised.
	\par (b) MAC_SELF_SET_RADIO_SLEEP: 
	This message is sent by the MAC module to itself, and is used to put the radio to sleep.
	\par (c) MAC_SELF_WAKEUP_RADIO: 
	This message is sent by the MAC module to itself, and is used to wake the radio up.
	\par (d) NET_FRAME: 
	This message is sent by the network module to the MAC module, and indicates the arrival of a packet to be transmitted from the network layer.
	\par (e) MAC_SELF_PUSH_TX_BUFFER: 
	This message is sent by the MAC module to itself, and is used to push the received frame into the transmission buffer.
	\par (f) MAC_SELF_INITIATE_TX: 
	This message is sent by the MAC module to itself, and is used to initiate a transmission by disabling duty cycling, switching the radio on, and initiating a carrier sense.
	\par (g) MAC_SELF_PERFORM_CARRIER_SENSE: 
	This message is sent by the MAC module to itself, and is used to start a carrier sense.
	\par (h) RADIO_2_MAC_SENSED_CARRIER: 
	This message is sent by the radio module to the MAC module, and indicates that the carrier is  busy.
	\par (i) MAC_SELF_EXIT_CARRIER_SENSE: 
	This message is sent by the MAC module to itself, and indicates that the carrier busy message was not received from the radio, and the carrier is hence free.
	\par (j) MAC_SELF_CHECK_TX_BUFFER: 
	This message is sent by the MAC module to itself, and is used to pop the frame at the head of the transmission buffer queue, and send it to the radio module.
	\par (k) RADIO_2_MAC_STARTED_TX: 
	This message is sent by the radio module to the MAC module, and indicates that the radio has started transmission of the frame into the channel.
	\par (l) RADIO_2_MAC_STOPPED_TX: 
	This message is sent by the radio module to the MAC module to indicate completion of the transmission operation in the physical layer.
	\par (m) MAC_FRAME: 
	This message is sent by the radio module to the MAC module when a frame is received by the radio module. This frame is processed and sent to the network layer.
	\par (n) RESOURCE_MANAGER_OUT_OF_ENERGY
	This message is sent by the resource manager module to the MAC module when the node rns out of battery. Disable node when this occurs.

	\param msg
	This parameter contains the message received by the MAC module.
 */
void SpeckMacModule::handleMessage (cMessage *msg)
{
	int msgKind = msg->kind();
	
	if ((disabled == TRUE) && (msgKind != APP_NODE_STARTUP))
	{
		delete msg;
		msg = NULL;
		return;
	}
	
	switch (msgKind)
	{
		case APP_NODE_STARTUP:
		{
			disabled = FALSE; // enable the Node's MAC layer.
				
			dutyCycleWakeupMsg = new MAC_ControlMessage("wake_up_radio", MAC_SELF_WAKEUP_RADIO);
			scheduleAt(simTime(), dutyCycleWakeupMsg); // Switch to wake up mode  now. Sleep automatically scheduled.
			
			break;
		}
		
		case MAC_SELF_SET_RADIO_SLEEP:
		{
			dutyCycle(SLEEP);
			break;
		}
		
		case MAC_SELF_WAKEUP_RADIO:
		{
			dutyCycle(WAKEUP);
			break;
		}
		
		case NET_FRAME:
		{
			handleNetworkLayerFrame(msg);
			break;
		}
		
		case MAC_FRAME_SELF_PUSH_TX_BUFFER:
		{
			pushFrameIntoBuffer(msg);
			break;
		}
		
		case MAC_SELF_INITIATE_TX:
		{
			// disable duty cycling, start radio up.
			if (macState == MAC_STATE_DEFAULT)
			{
				macState = MAC_STATE_TRY_TX;
				if ( (dutyCycleWakeupMsg != NULL) && (dutyCycleWakeupMsg->isScheduled()) )
				{
					cancelAndDelete(dutyCycleWakeupMsg);
					dutyCycleWakeupMsg = NULL;
				}
				if ( (dutyCycleSleepMsg != NULL) && (dutyCycleSleepMsg->isScheduled()) )
				{
					cancelAndDelete(dutyCycleSleepMsg);
					dutyCycleSleepMsg = NULL;
				}
				
				// set radio to listen.
				setRadioState(MAC_2_RADIO_ENTER_LISTEN, 0.001 * dblrand());
				
				CASTALIA_DEBUG << "\n[SpeckMAC_"<< self << "] t=" << simTime() << ": Init TX;  Mac State=" << macState;
				initiateCarrierSense();
			}
			
			break;
		}
		
		case MAC_SELF_PERFORM_CARRIER_SENSE:
		{
			performCarrierSense();
			// If carrier is free, MAC_SELF_EXIT_CARRIER_SENSE happens. if not, RADIO_2_MAC_CARRIER_SENSED occurs.
			break;
		}
		
		// Carrier busy
		case RADIO_2_MAC_SENSED_CARRIER:
		{
			carrierBusy();
			break;
		}
		
		// Carrier free.
		case MAC_SELF_EXIT_CARRIER_SENSE:
		{
			carrierFree();
			break;
		}
		
		case MAC_SELF_CHECK_TX_BUFFER:
		{
			sendData();
			break;
		}
		
		case RADIO_2_MAC_STARTED_TX:
		{	
			if(macState == MAC_STATE_CARRIER_SENSING)
			{
				macState = MAC_STATE_DEFAULT;
				if(printStateTransitions)
				{
					CASTALIA_DEBUG << "\n[SpeckMAC_" << self <<"] t= " << simTime() << ": State changed to MAC_STATE_DEFAULT (RADIO_2_MAC_STARTED_TX received when MAC_STATE_CARRIER_SENSING)";
				}

				scheduleAt(simTime(), new MAC_ControlMessage("check schedTXBuffer buffer", MAC_SELF_CHECK_TX_BUFFER));
			}
			else if(macState == MAC_STATE_DEFAULT)
			{
				CASTALIA_DEBUG << "\n[SpeckMAC_" << self <<"] t= " << simTime() << "; Start TX";
				macState = MAC_STATE_TX;
				if(printStateTransitions)
				{
					CASTALIA_DEBUG << "\n[SpeckMAC_" << self <<"] t= " << simTime() << ": State changed to MAC_STATE_TX (RADIO_2_MAC_STARTED_TX received when MAC_STATE_DEFAULT)";
				}
			}
			break;
		}
	
		case RADIO_2_MAC_STOPPED_TX:
		{
			finishDataTransmission();
			break;
		}
		
		// packet received from radio.
		case MAC_FRAME:
		{
			MAC_GenericFrame *rcvFrame;
			rcvFrame = check_and_cast<MAC_GenericFrame*>(msg);
#ifdef STATISTICS
			numRecd ++;
#endif	
			CASTALIA_DEBUG << "\n[SpeckMAC_" << self << "] t= " << simTime() << ": Rx Pkt";
			
			macState = MAC_STATE_DEFAULT;
			
			// Cancel currently scheduled sleep
			if ( (dutyCycleWakeupMsg != NULL) && (dutyCycleWakeupMsg->isScheduled()) )
			{
				cancelAndDelete(dutyCycleWakeupMsg);
			}
			if ( (dutyCycleSleepMsg != NULL) && (dutyCycleSleepMsg->isScheduled()) )
			{
				cancelAndDelete(dutyCycleSleepMsg);
			}
			
			// Sleep now.
			dutyCycleSleepMsg = new MAC_ControlMessage("put_radio_to_sleep", MAC_SELF_SET_RADIO_SLEEP);
			scheduleAt(simTime(), dutyCycleSleepMsg); 	
#ifdef DEBUG
			if ( (self == 6) || (self == 5) )
				CASTALIA_DEBUG << "\n[SpeckMAC_"<<self<<"] t="<< simTime() << ": Sleep now"; 
#endif	
			int destinationID = rcvFrame->getHeader().destID;
				
			Network_GenericFrame *netDataFrame; // No need to create a new message because of the decapsulation: netDataFrame = new Network_GenericFrame("Network frame MAC->Network", NET_FRAME);
	
			// decaspulate the received MAC frame and create a valid Network_GenericFrame
			netDataFrame = check_and_cast<Network_GenericFrame *>(rcvFrame->decapsulate());
	
			// Send the App_GenericDataPacket message to the Application module
			send(netDataFrame, "toNetworkModule");
			
			break;
		}
		
		case RESOURCE_MGR_OUT_OF_ENERGY:
		{
			disabled = 1;
			break;
		}
			
	} // end of switch case.
	
	delete msg;
	msg = NULL;
}

/*!
	\brief Performs duty cycling.
	
	This method schedule duty cycling. Every sleepInterval seconds, it switches on and schedules a wakeup message after listenInterval seconds, and vice versa. 

	\param typeID
	This parameter takes two values, SLEEP or WAKEUP, which specifies whether the radio is switched on, or put to sleep mode.
 */
void SpeckMacModule::dutyCycle(int typeID)
{
	if (typeID == SLEEP)
	{	
		if (macState == MAC_STATE_EXPECTING_RX)
		{
			if ( (self == 6) || (self == 5) )
				CASTALIA_DEBUG << "\n[SpeckMAC_"<<self<<"] t = "<< simTime() << ": Rx failed.";
			macState = MAC_STATE_DEFAULT;
		}
		
		if (macState == MAC_STATE_DEFAULT) // if its not txing, expecting an rx, or carrier sensing
		{
#ifdef DEBUG
			if ( (self == 6) || (self == 5) )
				CASTALIA_DEBUG << "\n[SpeckMAC_"<<self<<"] t = "<< simTime() << ": Radio sleep.";
#endif
			
			setRadioState(MAC_2_RADIO_ENTER_SLEEP); // switch to sleep mode.
				
			if ( (dutyCycleWakeupMsg != NULL) && (dutyCycleWakeupMsg->isScheduled()) )
			{
				cancelAndDelete(dutyCycleWakeupMsg);
			}
			
			dutyCycleWakeupMsg = new MAC_ControlMessage("wake_up_radio", MAC_SELF_WAKEUP_RADIO);
	
			scheduleAt(simTime() + DRIFTED_TIME(sleepInterval), dutyCycleWakeupMsg);
		}
		else
		{
			CASTALIA_DEBUG << "\n[SpeckMAC_"<<self<<"] t = "<< simTime() << ": Radio sleep FAILED.";
		}
		dutyCycleSleepMsg = NULL;
	}
	else if (typeID == WAKEUP)
	{		
#ifdef DEBUG
		if ( (self == 6) || (self == 5) )
			CASTALIA_DEBUG << "\n[SpeckMAC_"<<self<<"]t = "<< simTime() << ": Radio wakeup";
#endif
		
		if (macState == MAC_STATE_DEFAULT)
		{
			setRadioState(MAC_2_RADIO_ENTER_LISTEN);
			
			lastWakeupTime = simTime(); // This is the time the node wakes up.
			
			if ( (dutyCycleSleepMsg != NULL) && (dutyCycleSleepMsg->isScheduled()) )
			{
				cancelAndDelete(dutyCycleSleepMsg);
			}
			
			dutyCycleSleepMsg = new MAC_ControlMessage("put_radio_to_sleep", MAC_SELF_SET_RADIO_SLEEP);
			scheduleAt(simTime() + DRIFTED_TIME(listenInterval), dutyCycleSleepMsg); // Get radio to go to sleep
			
			initiateCarrierSense();
		}
		else
		{
			CASTALIA_DEBUG << "\n[SpeckMAC_"<<self<<"] t = "<< simTime() << ": Radio wakeup FAILED.";
		}
		
		dutyCycleWakeupMsg = NULL;
	}
}

/*!
	\brief Handles network layer packets received from the Network module.
	
	This method is executed whenever the MAC module receives a network layer frame (case NET_FRAME). It encapsulates the network layer packet into a MAC layer frame, sets the \b{doTx} flag to indicate that the module has to transmit the packet to the radio, and schedules a message that pushes the frame into the transmission buffer. It then schedules, after a random offset period which may be defined in the ini file, the initiation of transmission to the radio layer.
	
	\param msg
	This parameter holds the network layer packet received.
*/
void SpeckMacModule::handleNetworkLayerFrame(cMessage *msg)
{
	if (!BUFFER_IS_FULL)
	{
		Network_GenericFrame *rcvNetDataFrame = check_and_cast<Network_GenericFrame*>(msg);
		// Create the MACFrame from the Network Data Packet (encapsulation)	
		MAC_GenericFrame *dataFrame;
		
		char buff[50];
	
		sprintf(buff, "MAC Data frame (%f)", simTime());
		dataFrame = new MAC_GenericFrame(buff, MAC_FRAME);
		
		if(encapsulateNetworkFrame(rcvNetDataFrame, dataFrame))
		{
			doTx = TRUE; // indicate that tx has to be performed. This is because both Tx and Rx use Carrier sense.
			
			dataFrame->setKind(MAC_FRAME_SELF_PUSH_TX_BUFFER); // the dataFrame is a pointer of type MAC_GenericFrame*
			scheduleAt(simTime(), dataFrame);
			// (int) ((sleepInterval * radioDataRate * 1000) / ((rcvNetDataFrame->byteLength() + macFrameOverhead) * 8) );
			//pushFrameIntoBuffer(msg);
			
			scheduleAt(simTime() + DRIFTED_TIME(dblrand() * randomTxOffset), new MAC_ControlMessage("initiate a TX", MAC_SELF_INITIATE_TX));
		}
		else
		{
			cancelAndDelete(dataFrame);
			dataFrame = NULL;
			CASTALIA_DEBUG << "\n[SpeckMAC_" << self <<"] t= " << simTime() << ": WARNING: Network module sent to MAC an oversized packet...packet dropped!!\n";
		}
		
		rcvNetDataFrame = NULL;
		dataFrame = NULL;
	}
	
}

/*!
	\brief Initiates carrier sense.
	
	SpeckMAC performs carrier sense under two circumstances: (a) if the node is checking for packets in the medium, and (b) if the node requires to transmit, and needs to perform a Clear Channel Assessment (CCA) a priori. It schedules a MAC_SELF_PERFORM_CARRIER_SENSE message.
 */
void SpeckMacModule::initiateCarrierSense()
{
	// there is no point to do the following things when the Mac is already in TX mode and moreover if it is in MAC_STATE_EXPECTING_RX
	if ( (macState == MAC_STATE_DEFAULT) || (macState == MAC_STATE_TRY_TX) )
	{	
		if ( ((doTx == TRUE) && !(BUFFER_IS_EMPTY)) || (doTx == FALSE) ) // either packet to be transmitted, and buffer not empty, or no packet to  be transmitted, perform carrier sense
		{
			
			scheduleAt(simTime(), new MAC_ControlMessage("Enter carrier sense state MAC->MAC", MAC_SELF_PERFORM_CARRIER_SENSE));
			// perform carrier sense NOW!
#ifdef DEBUG
			if ( (self == 6) || (self == 5) )
				CASTALIA_DEBUG << "\n[SpeckMAC_" << self << "] t= " << simTime() << " Perform Carrier Sense.";
#endif
		}							  
		else if ( (doTx == TRUE) && (BUFFER_IS_EMPTY) )
		{
			CASTALIA_DEBUG << "\n[SpeckMAC_" << self << "] t= " << simTime() << ": WARNING: MAC_SELF_INITIATE_TX received but Mac Buffer is empty.\n";

			macState = MAC_STATE_DEFAULT;
			if(printStateTransitions)
			{
				CASTALIA_DEBUG << "\n[SpeckMAC_" << self <<"] t= " << simTime() << ": State changed to MAC_STATE_DEFAULT (MAC_SELF_INITIATE_TX received and buffer is empty)";
			}
			
			// Put node back to sleep; dont perform carrier sense.
			scheduleAt(simTime() + DRIFTED_TIME(listenInterval), new MAC_ControlMessage("put_radio_to_sleep", MAC_SELF_SET_RADIO_SLEEP));
		}
	}
	
}

/*!
	\brief Performs carrier sense.
	
	SpeckMAC performs carrier sense under two circumstances: (a) if the node is checking for packets in the medium, and (b) if the node requires to transmit, and needs to perform a Clear Channel Assessment (CCA) a priori. This function checks if the radio's carrier sense is valid. If it is not, it reschedules itself. If it is, it orders the radio to sense the carrier for a period defined in the \e.ini file, and schedules a carrier free message for a time shortly after the aforementioned period.
 */
void SpeckMacModule::performCarrierSense()
{
	if ( (macState == MAC_STATE_DEFAULT) || (macState == MAC_STATE_TRY_TX) )
	{		
		int isCarrierSenseValid_ReturnCode; // during this procedure we check if the carrier sense indication of the Radio is valid.
		isCarrierSenseValid_ReturnCode = radioModule->isCarrierSenseValid();

		if(isCarrierSenseValid_ReturnCode == 1) // carrier sense indication of Radio is Valid
		{
			// send the delayed message with the command to perform Carrier Sense to the radio
			MAC_ControlMessage *csMsg = new MAC_ControlMessage("carrier sense strobe MAC->radio", MAC_2_RADIO_SENSE_CARRIER);
			
			csMsg->setSense_carrier_interval(CARRIER_SENSE_INTERVAL); // Add a random value.
			send(csMsg, "toRadioModule"); // Send message to radio module NOW.

			selfExitCSMsg = new MAC_ControlMessage("Exit carrier sense state MAC->MAC", MAC_SELF_EXIT_CARRIER_SENSE);
			scheduleAt(simTime() + CARRIER_SENSE_INTERVAL + epsilon, selfExitCSMsg);

			macState = MAC_STATE_CARRIER_SENSING; // Indicate that SpeckMAC will now perform carrier sensing.
			
			if(printStateTransitions)
				CASTALIA_DEBUG << "\n[SpeckMAC_" << self <<"] t= " << simTime() << ": State changed to MAC_STATE_CARRIER_SENSING (MAC_SELF_PERFORM_CARRIER_SENSE received)";
		}
		else // carrier sense indication of Radio is NOT Valid and isCarrierSenseValid_ReturnCode holds the cause for the non valid carrier sense indication. **This should not happen, as I switch the radio to listen a lot earlier.**
		{
			switch(isCarrierSenseValid_ReturnCode)
			{
				case RADIO_IN_TX_MODE:
				{
					// send the packet (+ the precending beacons) to the Radio Buffer  by sending a message to ourselves
					scheduleAt(simTime(), new MAC_ControlMessage("check schedTXBuffer buffer", MAC_SELF_CHECK_TX_BUFFER));
					break;
				}

				case RADIO_SLEEPING:
				{
					// wake up the radio
					setRadioState(MAC_2_RADIO_ENTER_LISTEN);
					// send to ourselves a MAC_SELF_PERFORM_CARRIER_SENSE with delay equal to the time needed by the radio to have a valid CS indication after switching to LISTENING state
					scheduleAt(simTime() + DRIFTED_TIME(radioDelayForValidCS) + epsilon, new MAC_ControlMessage("Enter carrier sense state MAC->MAC", MAC_SELF_PERFORM_CARRIER_SENSE));
					break;
				}

				case RADIO_NON_READY:
				{
					//send to ourselves a MAC_SELF_PERFORM_CARRIER_SENSE with delay equal to the time needed by the radio to have a valid CS indication after switching to LISTENING state
					scheduleAt(simTime() + DRIFTED_TIME(radioDelayForValidCS), new MAC_ControlMessage("Enter carrier sense state MAC->MAC", MAC_SELF_PERFORM_CARRIER_SENSE));

					break;
				}

				default:
				{
					CASTALIA_DEBUG << "\n[SpeckMAC_"<< self <<"] t= " << simTime() << ": WARNING: In MAC module, radioModule->isCarrierSenseValid(reasonNonValid) return invalid reasonNonValid.\n";
					break;
				}
			}//end_switch
		}
	}
}


/*!
	\brief Handles channel busy condition.
	
	This method is called if the channel is busy. There are two possibilities:
	\par Check for frames in the medium.
	In this case, the module switches to MAC_STATE_EXPECTING_RX, and waits for a frame for a maximum time equal to the length of two maximum sized MAC frames.

	\par Perform CCA to transmit.
	If blocking send is enabled, the module defers transmission and waits for a frame for a maximum time equal to the length of two maximum sized MAC frames. If not, the frame is discarded, and the node switches to receiving mode as described above. 

	\note Blocking send is enabled by default.
 */
void SpeckMacModule::carrierBusy()
{
	// The channel is not free; therefore, the impending carrier sense exit message must be cancelled in order not to give the false impression that the channel is free.
#ifdef DEBUG
	if ( (self == 6) || (self == 5) )
		CASTALIA_DEBUG <<"\n[SpeckMAC_"<< self <<"] t=" << simTime() << ": Carrier Busy, MAC State = " << macState;
#endif
	
	if (( selfExitCSMsg != NULL ) && (  selfExitCSMsg->isScheduled() ))
	{
		cancelAndDelete(selfExitCSMsg);
		selfExitCSMsg = NULL;
	}
	
	if ( (macState == MAC_STATE_CARRIER_SENSING) || (macState == MAC_STATE_DEFAULT) ) // very very debug. May cause side-effects.
	{
		
		if (doTx == TRUE) // pkt to be Txed
		{
			// Try retransmitting after sleepInterval seconds; so that all redundant retransmissions have cleared.
			// scheduleAt(simTime() + DRIFTED_TIME(sleepInterval), new MAC_ControlMessage("try transmitting after backing off", MAC_SELF_INITIATE_TX));
			
			
#ifndef BLOCKING
		// If NOT blocking send; i.e., repeated tries till failure; disabled.
#ifdef DEBUG
		CASTALIA_DEBUG <<"\n[SpeckMAC_"<< self <<"] t=" << simTime() << ": Pkt send failed, and pkt buffer size = " << getTXBufferSize() << "; Mac State =" << macState ;
#endif
				
			MAC_GenericFrame* rubbish = popTxBuffer();
				
				// If everything has been popped out
			if (getTXBufferSize() == 0)
			{
				doTx = FALSE;
			}
				
			delete rubbish;
			rubbish = NULL;
				
			macState = MAC_STATE_EXPECTING_RX;
				
			if (printStateTransitions)
			{
				CASTALIA_DEBUG <<"\n[SpeckMAC_"<< self <<"] t=" << simTime() <<"; Mac State =" << macState << " i.e. changed to MAC_EXPECTING_RX";
			}
				
				// Cancel currently scheduled sleep
			if ( (dutyCycleWakeupMsg != NULL) && (dutyCycleWakeupMsg->isScheduled()) )
			{
				cancelAndDelete(dutyCycleWakeupMsg);
			}
			if ( (dutyCycleSleepMsg != NULL) && (dutyCycleSleepMsg->isScheduled()) )
			{
				cancelAndDelete(dutyCycleSleepMsg);
			}
				
#else
				// ********************** THIS HAS BEEN TESTED, AND WORKS ****************************************** 
				// If blocking send; don't delete the packet.
#ifdef DEBUG
			CASTALIA_DEBUG <<"\n[SpeckMAC_"<< self <<"] t=" << simTime() << ": Pkt send failed, and pkt buffer size = " << getTXBufferSize() << "; retry after sleeping for sleepInterval. Mac State =" << macState ;
#endif		
			// Cancel currently scheduled sleep, and schedule another sleep for after the length of two longest possible frames.
			// This is to ensure that the node doesn't have the radio permanently turned on by random noise, or incompletely received packets.
			if ( (dutyCycleWakeupMsg != NULL) && (dutyCycleWakeupMsg->isScheduled()) )
			{
				cancelAndDelete(dutyCycleWakeupMsg);
			}
			if ( (dutyCycleSleepMsg != NULL) && (dutyCycleSleepMsg->isScheduled()) )
			{
				cancelAndDelete(dutyCycleSleepMsg);
			}
			
			macState = MAC_STATE_EXPECTING_RX;
				
			if (printStateTransitions)
			{
				CASTALIA_DEBUG <<"\n[SpeckMAC_"<< self <<"] t=" << simTime() <<"; Mac State =" << macState << " i.e. changed to MAC_EXPECTING_RX";
			}
			
			dutyCycleSleepMsg = new MAC_ControlMessage("put_radio_to_sleep", MAC_SELF_SET_RADIO_SLEEP);
			scheduleAt(simTime() + DRIFTED_TIME((double) (2 * maxMacFrameSize * 8 / (1000.0 * radioDataRate))), dutyCycleSleepMsg); 	
#ifdef DEBUG
			if ( (self == 5) || (self == 6) )
				CASTALIA_DEBUG << "\n[SpeckMAC_"<<self<<"] t="<< simTime() << ": Sleep after " << ( (2 *maxMacFrameSize * 8) / (1000.0 * radioDataRate) );	
#endif
			
#endif	// blocking		
		}
		else // This was a carrier sense to see if medium had packets.
		{
			// Cancel currently scheduled sleep, and schedule another sleep for after the length of two longest possible frames.
			// This is to ensure that the node doesn't have the radio permanently turned on by random noise, or incompletely received packets.
			if ( (dutyCycleWakeupMsg != NULL) && (dutyCycleWakeupMsg->isScheduled()) )
			{
				cancelAndDelete(dutyCycleWakeupMsg);
			}
			if ( (dutyCycleSleepMsg != NULL) && (dutyCycleSleepMsg->isScheduled()) )
			{
				cancelAndDelete(dutyCycleSleepMsg);
			}
			
			macState = MAC_STATE_EXPECTING_RX;
			
			
			if (printStateTransitions)
			{
				CASTALIA_DEBUG <<"\n[SpeckMAC_"<< self <<"] t=" << simTime() <<"; Mac State =" << macState << " i.e. changed to MAC_EXPECTING_RX";
			}

			// put radio to sleep.
			dutyCycleSleepMsg = new MAC_ControlMessage("put_radio_to_sleep", MAC_SELF_SET_RADIO_SLEEP);
			scheduleAt(simTime() + DRIFTED_TIME((double) (2 * maxMacFrameSize * 8 / (1000.0 * radioDataRate))), dutyCycleSleepMsg); 	
			
			if ( (self == 5) || (self == 6) )
				CASTALIA_DEBUG << "\n[SpeckMAC_"<<self<<"] t="<< simTime() << ": Sleep after " << ( (2 *maxMacFrameSize * 8) / (1000.0 * radioDataRate) );
		}
	}
}

/*!
	\brief Handles channel free condition.
	
	This method is called if the channel is free. There are two possibilities:
	\par Check for frames in the medium.
	In this case, the module performs carrier sense again, if there is time left in the listenInterval to perform another carrier sense. This is a modification to SpeckMAC to deal with packet-based radios. The original implementation on the Prospeckz IIK used the CC2420 radio's stream mode.

	\par Perform CCA to transmit.
	The node transmits the frame.
	\todo Consider perform carrier senses before transmitting, in case the earlier carrier senses fell on intervals between successive packets in a redundant data frame send.
	\bug Sometimes it is possible for two nodes to be perfectly synchronised. This is dealt with by the random offset introduced to MAC_SELF_INITIATE_TX. This is not entirely effective. However, by adding an offset before transmission in the application layer, delivery ratios equal to 100% may be achieved.
 */
void SpeckMacModule::carrierFree()
{
	if ( (self == 6) || (self == 5) )
		CASTALIA_DEBUG << "\n[SpeckMAC_" << self <<"] t= " << simTime() << ": Carrier Free";
	if (macState == MAC_STATE_CARRIER_SENSING)
	{
		macState = MAC_STATE_DEFAULT;

		if(printStateTransitions)
		{
			CASTALIA_DEBUG << "\n[SpeckMAC_" << self <<"] t= " << simTime() << ": State changed to MAC_STATE_DEFAULT (MAC_SELF_EXIT_CARRIER_SENSE received when MAC_STATE_CARRIER_SENSING) because Carrier is FREE!!";
		}
		
		if (doTx == TRUE) // if pkt to be Txed, schedule a message NOW that will check the Tx buffer for transmission
		{
			// This is because the node could be in wakeup state and already performing carrier sense when a message comes through.
			//if (self == 6)
			//{
#ifdef DEBUG
			CASTALIA_DEBUG << "\n[SpeckMAC_" << self <<"] t= " << simTime() << ": Retxing";
#endif
			//}
			if ( (dutyCycleWakeupMsg != NULL) && (dutyCycleWakeupMsg->isScheduled()) )
			{
				cancelAndDelete(dutyCycleWakeupMsg);
				dutyCycleWakeupMsg = NULL;
			}
			if ( (dutyCycleSleepMsg != NULL) && (dutyCycleSleepMsg->isScheduled()) )
			{
				cancelAndDelete(dutyCycleSleepMsg);
				dutyCycleSleepMsg = NULL;
			}
			
			scheduleAt(simTime(), new MAC_ControlMessage("check schedTXBuffer buffer", MAC_SELF_CHECK_TX_BUFFER));
		}
		// If not, this was a carrier sense just to see if the medium had packets. It does not; so run another carrier sense.
		else
		{
			double timeLeftListening;
			timeLeftListening = listenInterval - (simTime() - lastWakeupTime);
			if (timeLeftListening > (radioDelayForValidCS + CARRIER_SENSE_INTERVAL) ) // If there's time for another carrier sense, do IT!
			{
				if ( (self == 5) || (self == 6) )
					CASTALIA_DEBUG <<"\n[SpeckMAC_"<< self <<"] t=" << simTime() << ": Redo carrier sense";
				initiateCarrierSense();
			}
		}
	}	
	
	selfExitCSMsg = NULL; // Cancel the Carrier Sense exit message.
}

/*!
	\brief Sends data to the radio module.
	
	This method is called \b if the channel is free. It is used to transmit the frame at the head of the transmit buffer. The SpeckMAC-D algorithm performs most of its work in this method. It determines the number of frames that can be transmitted in the sleep interval,\b n, and sends \b n \b + \b 1 redundant frames.
	\todo Get rid of the else condition. It shouldn't happen.
	\bug Since sleepInterval/packetSize is not always an integer, we round up! This could result in multiple packet receives. But this is easier to deal with, and less harmful than the other possibility - packet loss.
 */
void SpeckMacModule::sendData()
{
	if (!(BUFFER_IS_EMPTY))
	{
		if ( (macState == MAC_STATE_TX) || (macState == MAC_STATE_DEFAULT) )
		{
			// SEND THE DATA FRAME TO RADIO BUFFER repeatedly; until the buffer is empty.
		
			MAC_GenericFrame *dataFrame, *dupFrame;
			dataFrame = popTxBuffer();
#ifdef STATISTICS				
			numSent ++;
#endif	
			for (int i = 0; i <= redundancy; i++) // Send multiple packets - redundancy + 1. Send them back to back.
			{
				dupFrame = (MAC_GenericFrame *)dataFrame->dup();
				sendDelayed(dupFrame, DRIFTED_TIME(i*dataTXtime) ,"toRadioModule");
				setRadioState(MAC_2_RADIO_ENTER_TX, DRIFTED_TIME(i*dataTXtime) + epsilon); // Remove epsilon.??
			}
 			
			delete dataFrame;
			dataFrame = NULL;
			
			// If the buffer is still not empty; i.e., the node has more packets to send, sleep for a short period = length of data packet + epsilon, so u restart carrier sense after that
			
			// check to see if we must schedule extra transmissions
		}
		else 
		{
			// The application will never enter this, as SELF_CHECK_TX_BUFFER is called only if buffer not empty. But as a safeguard. Doesn't work properly cuz I dont think it matters. ;)
			
			macState = MAC_STATE_DEFAULT;
			if(printStateTransitions)
			{
				CASTALIA_DEBUG << "\n[SpeckMAC_" << self <<"] t= " << simTime() << ": State changed to MAC_STATE_DEFAULT (MAC_SELF_CHECK_TX_BUFFER received and buffer is empty). ERROR: THIS CODE SHOULD NOT BE EXECUTED.";
			}
			setRadioState(MAC_2_RADIO_ENTER_SLEEP); // for a period equal to one guard period, there is no transmission.
				
			// dutyCycleWakeupMsg = new MAC_ControlMessage("wake_up_radio", MAC_SELF_WAKEUP_RADIO);
			// scheduleAt(simTime() + DRIFTED_TIME(sleepInterval), dutyCycleWakeupMsg);
			
		}
	}
}

/*!
	\brief Mop-up tasks after data transmission
	
	This method is called when the radio module completes transmission, and schedules additional transmissions if necessary. Additional transmissions are carried out after a guard period, to prevent a given node locking the channel.
	\todo Test with multiple packets per node per try; i.e., at higher data rates.
 */
void SpeckMacModule::finishDataTransmission()
{
	if(macState == MAC_STATE_TX)
	{
		macState = MAC_STATE_DEFAULT;
		if(printStateTransitions)
		{
			CASTALIA_DEBUG << "\n[SpeckMAC_" << self <<"] t= " << simTime() << ": State changed to MAC_STATE_DEFAULT (RADIO_2_MAC_STOPPED_TX received when MAC_STATE_TX); i.e., transmission complete";
		}
				
		CASTALIA_DEBUG << "\n[SpeckMAC_" << self <<"] t= " << simTime() << ": Put radio to sleep till next SELF_INITIATE_TX or WAKEUP";
								
		if (!(BUFFER_IS_EMPTY))
		{
			CASTALIA_DEBUG << "\n[SpeckMAC_"<<self<<"] t= " << simTime() << ": Schedule additional transmissions";
				// Which is, like, now.
			
			scheduleAt(simTime()+ DRIFTED_TIME(dataTXtime + epsilon), new MAC_ControlMessage("check schedTXBuffer buffer", MAC_SELF_INITIATE_TX)); // restart carrier sense after guard period.
		}
		else // The buffer is empty.
		{
			doTx = FALSE; // Since the buffer is empty, there are no more transmissions to be performed.
			
			CASTALIA_DEBUG << "\n[SpeckMAC_"<<self<<"] t= " << simTime() << ": No additional transmissions. Tx complete. Wake up after sleeping for the guard period";
		}
	}
	
	// Put node to sleep. NOW!
	setRadioState(MAC_2_RADIO_ENTER_SLEEP);
	dutyCycleWakeupMsg = new MAC_ControlMessage("wakeup_radio", MAC_SELF_WAKEUP_RADIO);
	scheduleAt(simTime() + sleepInterval, dutyCycleWakeupMsg);
}
/*!
	\brief Reads parameters from the ini file.
	
	This method is called when the MAC module initialises. It loads parameters from the .ini file, into the appropriate class members.
 */
void SpeckMacModule::readIniFileParameters()
{
	printDebugInfo = par("printDebugInfo");
	printStateTransitions = par("printStateTransitions");
	
	sleepInterval = par("sleepInterval");
	listenInterval = par("listenInterval");
	randomTxOffset = par("randomTxOffset");
	
	maxMacFrameSize = par("maxMacFrameSize");
	macBufferSize = par("macBufferSize");
	macFrameOverhead = par("macFrameOverhead");
}

/*!
	\brief Set radio state.
	
	This method is called when the MAC module requires to change the radio state. Read the radio datasheet to know the time and energy required to switch states.
 */
void SpeckMacModule::setRadioState(MAC_ContorlMessageType typeID, double delay)
{
	if( (typeID != MAC_2_RADIO_ENTER_SLEEP) && (typeID != MAC_2_RADIO_ENTER_LISTEN) && (typeID != MAC_2_RADIO_ENTER_TX) )
	{
		opp_error("MAC attempt to set Radio into an unknown state. ERROR commandID");
	}

	MAC_ControlMessage * ctrlMsg = new MAC_ControlMessage("state command strobe MAC->radio", typeID);

	sendDelayed(ctrlMsg, delay, "toRadioModule");
}

/*!
	\brief Obtain frame at the head of the transmission buffer.
	
	This method is called when the MAC module requires to obtain a frame from the queue, and schedule it for transmission to the radio module. It also calculates the time taken to transmit the frame, and the number of redundant copies to be sent.
	
	\note Rounding is done to the next highest integer.
 */

MAC_GenericFrame* SpeckMacModule::popTxBuffer()
{
	if (tailTxBuffer == headTxBuffer) 
	{
		ev << "\nTrying to pop  EMPTY TxBuffer!!";
		tailTxBuffer--;  // reset tail pointer
		return NULL;
	}
	
	MAC_GenericFrame* dataFrame = NULL;
	
	dataFrame = schedTXBuffer[headTxBuffer];
	
	dataTXtime = ((double)(dataFrame->byteLength()+phyLayerOverhead) * 8.0 / (1000.0 * radioDataRate));
 
	redundancy = (int)( (sleepInterval / dataTXtime) + 0.5); // to round it off to the next highest integer.
	CASTALIA_DEBUG <<  "\n[SpeckMAC_"<<self<<"] t = "<< simTime() <<": Redundancy = "<< redundancy;
	headTxBuffer = (++headTxBuffer)%(MAC_BUFFER_ARRAY_SIZE);
	
	return dataFrame;
}

/*!
	\brief Get the size of the transmission buffer.
*/
int SpeckMacModule::getTXBufferSize(void)
{
	int size = tailTxBuffer - headTxBuffer;
	if ( size < 0 )
		size += MAC_BUFFER_ARRAY_SIZE;

	return size;
}

/*!
	\brief Push frame into buffer.
	
	Create a copy of the MAC frame generated from the received network packet, and push it into the buffer.
*/
void SpeckMacModule::pushFrameIntoBuffer(cMessage *msg)
{
	if(!(BUFFER_IS_FULL))
	{
		// CASTALIA_DEBUG << "[SpeckMAC_" << self << "] t=" << simTime() << ": Pushing frame into buffer\n";
		MAC_GenericFrame *dataFrame = check_and_cast<MAC_GenericFrame*>(msg);

				// create a new copy of the message because dataFrame will be deleted outside the switch statement
		MAC_GenericFrame *duplMsg = (MAC_GenericFrame *)dataFrame->dup(); //because theFrame will be deleted after the main switch in the handleMessage()
		pushBuffer(duplMsg);
		
	}
	else
	{
		MAC_ControlMessage *fullBuffMsg = new MAC_ControlMessage("MAC buffer is full Radio->Mac", MAC_2_NETWORK_FULL_BUFFER);

		send(fullBuffMsg, "toNetworkModule");
				
		CASTALIA_DEBUG  << "\n[SpeckMAC_"<< self <<"] t= " << simTime() << ": WARNING: SchedTxBuffer FULL!!! Network frame is discarded.\n";
	}
}

/*!
	\brief Push frame into the transmission buffer.
*/
int SpeckMacModule::pushBuffer(MAC_GenericFrame *theFrame)
{
	if(theFrame == NULL)
	{
		CASTALIA_DEBUG << "\n[SpeckMAC_" << self << "] t= " << simTime() << ": WARNING: Trying to push NULL MAC_GenericFrame to the MAC_Buffer!!\n";
		return 0;
	}

	tailTxBuffer = (++tailTxBuffer)%(MAC_BUFFER_ARRAY_SIZE); //increment the tailTxBuffer pointer. If reached the end of array, then start from position [0] of the array

	if (tailTxBuffer == headTxBuffer)
	{
		// reset tail pointer
		if(tailTxBuffer == 0)
		{
			tailTxBuffer = MAC_BUFFER_ARRAY_SIZE-1;
		}
		else
		{
			tailTxBuffer--;
		}
		CASTALIA_DEBUG << "\n[SpeckMAC_" << self << "] t= " << simTime() << ": WARNING: SchedTxBuffer FULL!!! value to be Tx not added to buffer\n";
		return 0;
	}

	
	theFrame->setKind(MAC_FRAME);
	
	if (tailTxBuffer==0)
	{
		schedTXBuffer[MAC_BUFFER_ARRAY_SIZE-1] = theFrame;
	}
	else
	{
		schedTXBuffer[tailTxBuffer-1] = theFrame;
	}
	
	return 1;
}

/*!
	\brief Encapsulates the network frame into a MAC frame.
 */
int SpeckMacModule::encapsulateNetworkFrame(Network_GenericFrame *networkFrame, MAC_GenericFrame *retFrame)
{
	int totalMsgLen = networkFrame->byteLength() + macFrameOverhead;
	if(totalMsgLen > maxMacFrameSize)
		return 0;
	retFrame->setByteLength(macFrameOverhead); //networkFrame->byteLength() extra bytes will be added after the encapsulation

	retFrame->getHeader().srcID = self;
	
	retFrame->getHeader().destID = resolvDestination(networkFrame->getHeader().destCtrl.c_str());
	
	retFrame->getHeader().frameType = MAC_PROTO_DATA_FRAME;

	Network_GenericFrame *dupNetworkFrame = check_and_cast<Network_GenericFrame *>(networkFrame->dup());
	retFrame->encapsulate(dupNetworkFrame);

	return 1;
}

int SpeckMacModule::resolvDestination(const char *routingDestination)
{
	string tmpParDestination(routingDestination);
	char buff[256];
	
	sprintf(buff, "%i" , BROADCAST_ADDR);
	string broadcastAddr(buff);
	
	
	if(tmpParDestination.compare(broadcastAddr) == 0)
	{
		return BROADCAST_ADDR;
	}
	else
	{
		return atoi(routingDestination);
	}
}
