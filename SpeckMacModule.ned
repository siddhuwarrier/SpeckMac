simple SpeckMacModule

parameters:
	printDebugInfo	:	bool,
	printStateTransitions	:	bool,
	sleepInterval	:	numeric,
	listenInterval	:	numeric,
	maxMacFrameSize	:	const,
	randomTxOffset	:		numeric,
	macBufferSize	:	const,
	macFrameOverhead	:	const;
gates:
	in: fromNetworkModule, fromRadioModule, fromCommModuleResourceMgr;
	out: toNetworkModule, toRadioModule;
	
endsimple