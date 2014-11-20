#include <iostream>
#include <iomanip>
#include <string>
#include <string.h>
#include <sstream>

#include <iocsh.h>
#include <registryFunction.h>
#include <epicsExport.h>
#include <epicsThread.h>
#include <dbFldTypes.h>
#include <aSubRecord.h>
#include <dbAddr.h>
#include <dbAccess.h>
#include <recGbl.h>

#include "asynDriver.h"
#include "evrTime.h"
#include "timeStampFifo.h"

using namespace		std;

int					DEBUG_TSFifo	= 1;

#ifndef NULL
#define NULL    0
#endif

using namespace		std;

/// Static TSFifo map by port name
static  map<string, TSFifo *>   ms_TSFifoMap;


// TimeStampFifo is the function that gets registered
// with asynDriver as the timeStampSource
static void TimeStampFifo(
	void					*	userPvt,
	epicsTimeStamp			*	pTimeStamp )
{
	const char		*	functionName	= "TimeStampFifo";
	if ( pTimeStamp == NULL )
		return;

	int			status	= -1;
	TSFifo	*	pTSFifo	= reinterpret_cast<TSFifo *>( userPvt );
	if ( pTSFifo != NULL )
		status = pTSFifo->GetTimeStamp( pTimeStamp );
	if ( status != asynSuccess )
	{
		epicsTimeGetCurrent( pTimeStamp );
		pTimeStamp->nsec |= PULSEID_INVALID;
		if ( DEBUG_TSFifo & 8 )
			printf( "Error %s: GetTimeStamp error %d for port %s\n",
					functionName, status, pTSFifo->GetPortName() );
	}
	return;
}


/// Constructor for TSFifo
TSFifo::TSFifo(
	const char	*	pPortName,
	aSubRecord	*	pSubRecord	)
	:	m_eventCode(	0				),
		m_genCount(		0				),
		m_delay(		0				),
		m_synced(		false			),
		m_pSubRecord(	pSubRecord		),
		m_portName(		pPortName		),
		m_idx(			0LL				),
		m_idxIncr(		MAX_TS_QUEUE	)
{
	AddTSFifo( this );
}
 

void TSFifo::AddTSFifo( TSFifo	*	pTSFifo )
{
	assert( FindByPortName( pTSFifo->m_portName ) == NULL );
	ms_TSFifoMap[ pTSFifo->m_portName ]	= pTSFifo;
}


void TSFifo::DelTSFifo( TSFifo	*	pTSFifo )
{
	ms_TSFifoMap.erase( pTSFifo->m_portName );
}
 

TSFifo	*	TSFifo::FindByPortName( const string & portName )
{
	map<string, TSFifo *>::iterator   it  = ms_TSFifoMap.find( portName );
	if ( it == ms_TSFifoMap.end() )
		return NULL;
	return it->second;
}


asynStatus TSFifo::RegisterTimeStampSource( )
{
	const char		*	functionName	= "TSFifo::RegisterTimeStampFifo";
	asynUser		*   pasynUser = pasynManager->createAsynUser( 0, 0 );
	asynStatus			status    = pasynManager->connectDevice( pasynUser, m_portName.c_str(), 0 );
	if ( status != asynSuccess )
	{
		printf( "Error %s: cannot connect to asyn port %s\n",
				functionName, m_portName.c_str() );
		return status;
	}
	status = pasynManager->registerTimeStampSource( pasynUser, this, TimeStampFifo );
	if ( status != asynSuccess )
	{
		printf( "Error %s: cannot register TimeStampSource for port %s\n",
				functionName, m_portName.c_str() );
		return status;
	}
	return asynSuccess;
}


int TSFifo::GetTimeStamp(
	epicsTimeStamp			*	pTimeStampRet )
{
	epicsTimeStamp				fifoTimeStamp;
	epicsUInt32					fidFifo	= PULSEID_INVALID;

//	TODO: Add locking as needed
//	epicsMutexLock( lock );

#if 0
	ErGetTicks(0, &erTicks);
	evrTimeGet(pTimeStamp, m_eventCode); 
	evrTimeGetFromPipeline(&m_current_timestamp,  evrTimeCurrent, m_current_modifier_a, 0,0,0,0);
#endif

	// First time or unsynced, m_idxIncr is MAX_TS_QUEUE, which
	// just gets the latest eventCode in the FIFO
	int	status	= evrTimeGetFifo( &fifoTimeStamp, m_eventCode, &m_idx, m_idxIncr );
//	int status	= evrTimeGetFifo( &fifoTimeStamp, m_eventCode, &m_idx, m_idxIncr );
	if ( status == 0 )
		m_idxIncr	= 1;

	fidFifo	= PULSEID( fifoTimeStamp );
	epicsTimeStamp		curTimeStamp;
	epicsTimeStamp		sysTimeStamp;
	if ( m_fidPrior == PULSEID_INVALID || m_fidDiffPrior == PULSEID_INVALID )
	{
		// No prior fiducial to compare with
		// See if the current system time matches the latest timeStamp for this eventCode
		evrTimeGet( &curTimeStamp, m_eventCode ); 
		epicsTimeGetCurrent( &sysTimeStamp ); 
		int	timeDiff	= PULSEID(sysTimeStamp) - PULSEID(curTimeStamp);
		if ( timeDiff > 1 )
		{
			//	Mark unsynced;
			m_idx					= 0LL;
			m_idxIncr				= MAX_TS_QUEUE;
			fifoTimeStamp.nsec	|= PULSEID_INVALID;
		}
	}
	else
	{	// See if we're synchronous w/ prior fiducials for this event code
		int fidDiff	= FID_DIFF( fidFifo, m_fidPrior );
		if ( fidDiff == m_fidDiffPrior )
			m_syncCount++;
		else
			m_syncCount	= 0;
		m_fidPrior		= fidFifo;
		m_fidDiffPrior	= fidDiff;
		if ( m_syncCount <= m_syncCountMin )
		{
			//	Mark unsynced;
			m_idx					= 0LL;
			m_idxIncr				= MAX_TS_QUEUE;
			fifoTimeStamp.nsec	|= PULSEID_INVALID;
		}
	}

//	epicsMutexUnlock (m_lock);

//	scanIoRequest(m_ioScanPvt);
	if ( m_pSubRecord != NULL )
	{
		dbCommon	*	pDbCommon	= reinterpret_cast<dbCommon *>( m_pSubRecord );
		dbScanLock(		pDbCommon	);
		dbProcess(		pDbCommon	);
		dbScanUnlock(	pDbCommon	);
	}

	return status;
}


epicsUInt32	TSFifo::Show( int level ) const
{
	printf( "TSFifo for port %s\n",	m_portName.c_str() );
	printf( "\tEventCode:\t%d\n",	m_eventCode );
	printf( "\tGeneration:\t%d\n",	m_eventCode );
	printf( "\tFidDelay:\t%d\n",	m_delay );
	printf( "\tSync Status:\t%s\n",	m_synced ? "Synced" : "Unsynced" );
	return 0;
}


extern "C" long TSFifo_Init(	aSubRecord	*	pSub	)
{
	static	bool	isInitialized	= false;

	// No need to do this more than once if we have more than
	// one sub record for TSFifo operations
	if ( isInitialized )
		return TSFifo_STS_OK;

	isInitialized	= true;

	return TSFifo_STS_OK;
}


//	TSFifo_Process
//	
//	Inputs:
//		A:	Port name, a stringIn or stringOut record
//		B:	Event code for timestamp
//		C:	Generation counter for EventCode timing, should increment on any timing change
//		D:	Expected delay in fiducials between the eventCode and the ts query
//
//	Outputs
//		A:	TSFifo Sync Status: 0 = unlocked, 1 = locked
//
extern "C" long TSFifo_Process( aSubRecord	*	pSub	)
{
	TSFifo		*   pTSFifo	= NULL;
	int				status	= 0;
	if ( DEBUG_TSFifo & 4 )
	{
		cout	<<	"TSFifo_Process: " << pSub->name	<<	endl;
	}

	if ( pSub->dpvt	 != NULL )
	{
		pTSFifo		= static_cast<TSFifo *>( pSub->dpvt );
	}
	else
	{
		epicsString	*	pPortName	= static_cast<epicsString *>( pSub->a );
		TSFifo		*   pTSFifo		= TSFifo::FindByPortName( pPortName->pString );
		if ( pTSFifo == NULL )
		{
			pTSFifo = new TSFifo( pPortName->pString, pSub );
		}
		if ( pTSFifo->m_pSubRecord != pSub )
		{
			printf( "Error %s: Unable to register as TSFifo port %s already registered to %s\n",
					pSub->name, pPortName->pString, pTSFifo->m_pSubRecord->name );
			return -1;
		}
		if ( pTSFifo->RegisterTimeStampSource() != asynSuccess )
		{
			printf( "Error %s: Unable to register timeStampSource for port %s\n",
					pSub->name, pPortName->pString );
			return -1;
		}
		pSub->dpvt = pTSFifo;
	}
	assert( pTSFifo != NULL );

	// Update timestamp FIFO parameters
	epicsInt32	*	pIntVal	= static_cast<epicsInt32 *>( pSub->b );
	if ( pIntVal != NULL )
		pTSFifo->m_eventCode	= *pIntVal;
	pIntVal	= static_cast<epicsInt32 *>( pSub->c );
	if ( pIntVal != NULL )
		pTSFifo->m_genCount		= *pIntVal;
	pIntVal	= static_cast<epicsInt32 *>( pSub->c );
	if ( pIntVal != NULL )
		pTSFifo->m_delay		= *pIntVal;

	// Update outputs
	pIntVal	= static_cast<epicsInt32 *>( pSub->vala );
	if ( pIntVal != NULL )
		*pIntVal	= pTSFifo->m_synced;

	return status;
}


// Register aSub functions
extern "C"
{
epicsRegisterFunction(	TSFifo_Init		);
epicsRegisterFunction(	TSFifo_Process	);
epicsRegisterFunction(	TimeStampFifo	);
epicsExportAddress( int, DEBUG_TSFifo	);
}

// Register shell callable functions with iocsh

//	Register ShowTSFifo
static const	iocshArg		ShowTSFifo_Arg0		= { "portName",	iocshArgString };
static const	iocshArg		ShowTSFifo_Arg1		= { "level",	iocshArgInt };
static const	iocshArg	*	ShowTSFifo_Args[2]	= { &ShowTSFifo_Arg0, &ShowTSFifo_Arg1 };
static const	iocshFuncDef	ShowTSFifo_FuncDef	= { "ShowTSFifo", 2, ShowTSFifo_Args };
static void		ShowTSFifo_CallFunc( const iocshArgBuf * args )
{
	TSFifo		*   pTSFifo		= TSFifo::FindByPortName( args[0].sval );
	if ( pTSFifo != NULL )
		pTSFifo->Show( args[1].ival );
	else
		printf( "Error: Unable to find TSFifo %s\n", args[0].sval );
}
static void ShowTSFifo_Register( void )
{
	iocshRegister( &ShowTSFifo_FuncDef, ShowTSFifo_CallFunc );
}
epicsExportRegistrar( ShowTSFifo_Register );


#if 0
epicsShareFunc int registerTimeStampFifo( const char *portName, const char * tsCallbackName )
{
	const char		*	functionName	= "registerTimeStampFifo";

	if ( !portName || ! tsCallbackName || (strlen(portName) == 0) || (strlen( tsCallbackName) == 0))
	{
		printf( "Usage: %s portName functionName\n", functionName );
		return -1;
	}

	timeStampCallback    pfnTSCallback = (timeStampCallback) registryFunctionFind(  tsCallbackName );
	if ( pfnTSCallback == NULL )
	{
		printf( "Error %s: Cannot find function \"%s\"\n", functionName,  tsCallbackName );
		return -1;
	}

	TSFifo		*	pTSFifo = TSFifo::FindByPortName( portName );
	if ( pTSFifo )
	{
		printf(	"Error %s: Port %s already has a timestamp source!\n",
				functionName, portName );
		return -1;
	}

	pTSFifo = new TSFifo( portName );
//	pTSFifo->pfnTSCallback = pfnTSCallback;

	asynUser		*   pasynUser = pasynManager->createAsynUser( 0, 0 );
	asynStatus			status    = pasynManager->connectDevice( pasynUser, portName, 0 );
	if ( status != 0 )
	{
		printf( "Error %s: cannot connect to port %s\n", functionName, portName );
		return -1;
	}
	pasynManager->registerTimeStampSource( pasynUser, static_cast<void *>(pTSFifo), pfnTSCallback );

	return 0;
}


static const iocshArg registerTimeStampFifoArg0 = {	"portName",			iocshArgString	};
static const iocshArg registerTimeStampFifoArg1 = {	"functionName",		iocshArgString	};
static const iocshArg * const registerTimeStampFifoArgs[] =
{
	&registerTimeStampFifoArg0, 
	&registerTimeStampFifoArg1
};
static const iocshFuncDef registerTimeStampFifoDef =
{
	"registerTimeStampFifo", 2, registerTimeStampFifoArgs
};
void registerTimeStampFifoCall( const iocshArgBuf *args )
{
	registerTimeStampFifo( args[0].sval, args[1].sval );
}

#endif
