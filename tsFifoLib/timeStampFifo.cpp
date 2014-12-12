#include <iostream>
#include <iomanip>
#include <string>
#include <string.h>
#include <stdlib.h>
#include <sstream>

#include <iocsh.h>
#include <registryFunction.h>
#include <epicsExport.h>
#include <epicsThread.h>
#include <dbFldTypes.h>
#include <aSubRecord.h>
#include <dbAddr.h>
#include <dbAccess.h>
#include <dbScan.h>
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
map<string, TSFifo *>   TSFifo::ms_TSFifoMap;


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
	aSubRecord	*	pSubRecord,
	TSPolicy		tsPolicy	)
	:	m_eventCode(	0				),
		m_genCount(		0				),
		m_genPrior(		0				),
		m_delay(		0.0				),
		m_synced(		false			),
		m_pSubRecord(	pSubRecord		),
		m_portName(		pPortName		),
		m_idx(			0LL				),
		m_idxIncr(		MAX_TS_QUEUE	),
		m_fidPrior(		PULSEID_INVALID	),
		m_fidDiffPrior(	0				),
		m_syncCount(	0				),
		m_syncCountMin(	1				),
		m_tsPolicy(		tsPolicy		)
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


/// Default GetTimestamp policy is to provide the best timestamp available
/// for the specified event code, TS_EVENT.  A pulse id is encoded into
/// the least significant 17 bits of the nsec timestamp field, as per
/// SLAC convention for EVR timestamps.   The pulse id is set to 0x1FFFF
/// if the timeStampFifo status is unsynced.
///   TS_EVENT	- Most recent timestamp for the specified event code, no matter how old
///   TS_SYNCED - If unsynced, no timestamp is provided and GetTimeStamp returns -1.
///   TS_BEST   - Provides a synced, pulse id'd timestamp for the specified event code
///				  if available.  If not, it provides the current time w/ the most recent
///				  fiducial pulse id.
int TSFifo::GetTimeStamp(
	epicsTimeStamp	*	pTimeStampRet )
{
	const char		*	functionName	= "TSFifo::GetTimeStamp";
	epicsTimeStamp		fifoTimeStamp;
	enum SyncType		{ CURRENT, FIDDIFF, FIFODLY, TOO_LATE, FAILED };
	enum SyncType		tySync	= FAILED;

	if ( pTimeStampRet == NULL )
		return -1;

//	TODO: Add locking as needed
//	epicsMutexLock( lock );

	if ( m_tsPolicy == TS_EVENT )
		return evrTimeGet( pTimeStampRet, m_eventCode); 

	epicsUInt32		fidFifo	= PULSEID_INVALID;
	int				fidDiff	= PULSEID_INVALID;

	// First time or unsynced, m_idxIncr is MAX_TS_QUEUE, which
	// just gets the most recent FIFO timestamp for that eventCode
	int	evrTimeStatus	= evrTimeGetFifo( &fifoTimeStamp, m_eventCode, &m_idx, m_idxIncr );
	if ( evrTimeStatus != 0 )
	{
		m_idxIncr	= MAX_TS_QUEUE;	// Reset FIFO
		m_synced	= false;
		return -1;
	}

	fidFifo		= PULSEID( fifoTimeStamp );
	m_idxIncr	= 1;

	// Get the last 360hz Fiducial
	int	fidLast	= evrGetLastFiducial();
	int	fidTgt	= fidLast;
	if( fidTgt < static_cast<int>( m_delay ) )
		fidTgt	+= FID_MAX;
	fidTgt -= m_delay;

	// Get the fiducial for this FIFO timestamp and compute
	// the target error and delta vs the prior fiducial
	if ( m_fidPrior != PULSEID_INVALID && fidFifo != PULSEID_INVALID )
		fidDiff	= FID_DIFF( fidFifo, m_fidPrior );
	int				tgtError = FID_DIFF( fidTgt, fidFifo );

	// Did we hit our target ficucial?
	if ( abs(tgtError) <= 2 )
	{
		// We're synced!
		m_synced	= true;
		m_syncCount++;
		tySync	= FIFODLY;
	}
	else
	{
		epicsTimeStamp		recentTimeStamp;
		evrTimeGet( &recentTimeStamp, m_eventCode ); 
		int	fidRecent	= PULSEID( recentTimeStamp );
		tgtError 		= FID_DIFF( fidTgt, fidRecent );
		if ( abs(tgtError) <= 1 )
		{
			// The FIFO index was stale, but the most recent entry is the one we want
			tySync		= CURRENT;
			m_idxIncr	= MAX_TS_QUEUE;	// Reset FIFO
			m_synced	= true;
			m_syncCount++;
		}
		else
		{	// See if we have a consistent fidDiff w/ prior samples
			if (	m_fidPrior != PULSEID_INVALID
				&&	m_fidDiffPrior == PULSEID_INVALID
				&&	m_fidDiffPrior == fidDiff )
			{
				tySync	= FIDDIFF;
				m_syncCount++;
				if( m_syncCount >= m_syncCountMin )
					m_synced	= true;
			}
			else
			{
				tySync		= FAILED;
				m_synced	= false;
				m_syncCount	= 0;
			}
		}
	}

	// Remember prior values
	m_fidPrior		= fidFifo;
	m_fidDiffPrior	= fidDiff;

	// Check for a generation change
	if( m_genPrior != m_genCount )
		m_synced	= false;
	m_genPrior		= m_genCount;

	if ( !m_synced )
	{
		//	Mark unsynced and reset FIFO selector
		m_idxIncr			= MAX_TS_QUEUE;
		fifoTimeStamp.nsec	|= PULSEID_INVALID;
	}

	if (	( DEBUG_TSFifo & 4 )
		|| (( DEBUG_TSFifo & 2 ) && (tySync == FIDDIFF)) )
	{
		char		acBuff[40];
		epicsTimeToStrftime( acBuff, 40, "%H:%M:%S.%04f", &fifoTimeStamp );
		printf( "%s: %-8s, %-8s, ts %s, fid 0x%X, fidFifo 0x%X, fidLast 0x%X, fidDiff %d, fidDiffPrior %d\n",
				functionName,
				( m_synced ? "Synced" : "Unsynced" ),
				(	tySync == CURRENT
				?	"CURRENT"
				:	(	tySync == FIDDIFF
		  			?	"FIDDIFF"
						:	(	tySync == FIFODLY
						?	"FIFODLY"
							:	(	tySync == TOO_LATE
								?	"TOO_LATE"
								:	"FAILED" ) ) ) ),
				acBuff, PULSEID(fifoTimeStamp ), fidFifo, fidLast,
				fidDiff, m_fidDiffPrior	);
	}
//	epicsMutexUnlock (m_lock);

	if ( m_pSubRecord != NULL )
	{
		dbCommon	*	pDbCommon	= reinterpret_cast<dbCommon *>( m_pSubRecord );
		scanOnce( pDbCommon );
	}

	if ( m_tsPolicy == TS_SYNCED && !m_synced )
		return -1;

	// If we have a pulse ID's timestamp, return it
	if ( PULSEID(fifoTimeStamp) != PULSEID_INVALID )
		*pTimeStampRet = fifoTimeStamp;
	else if ( m_tsPolicy == TS_BEST )
	{
		// Just get the latest 360Hz fiducial timestamp
		epicsTimeStamp		fidTimeStamp;
		evrTimeStatus	= evrTimeGet( &fidTimeStamp, 0 ); 
		*pTimeStampRet	= fidTimeStamp;
	}
	return evrTimeStatus;
}


epicsUInt32	TSFifo::Show( int level ) const
{
	printf( "TSFifo for port %s\n",	m_portName.c_str() );
	printf( "\tEventCode:\t%d\n",	m_eventCode );
	printf( "\tGeneration:\t%d\n",	m_genCount );
	printf( "\tFidDelay:\t%.3e\n",	m_delay );
	printf( "\tTS Policy:\t%s\n",	(	m_tsPolicy == TS_EVENT
									?	"EVENT"
									:	(	m_tsPolicy == TS_BEST
										?	"BEST" : "SYNCED" ) ) );
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
	if ( DEBUG_TSFifo & 8 )
	{
		cout	<<	"TSFifo_Process: " << pSub->name	<<	endl;
	}

	if ( pSub->dpvt	 != NULL )
	{
		// pTSFifo		= static_cast<TSFifo *>( pSub->dpvt );
		pTSFifo		= (TSFifo *) ( pSub->dpvt );
	}
	else
	{
		char	*	pPortName	= static_cast<char *>( pSub->a );
		if ( pPortName == NULL )
		{
			printf( "Error %s: NULL TSFifo port name\n", pSub->name );
			return -1;
		}
		if ( strlen(pPortName) == 0 )
		{
			printf( "Error %s: Empty TSFifo port name\n", pSub->name );
			return -1;
		}
		if ( strcmp(pPortName,"Unknown") == 0 )
		{
			if ( DEBUG_TSFifo & 2 )
				printf( "%s: Port name not available yet. Still %s\n", pSub->name, pPortName );
			return -1;
		}

		if ( DEBUG_TSFifo )
			printf( "%s: Attempting to register port name %s\n", pSub->name, pPortName );

		pTSFifo		= TSFifo::FindByPortName( pPortName );
		if ( pTSFifo == NULL )
		{
			if ( DEBUG_TSFifo )
				printf( "%s: Creating new TSFifo for port name %s\n", pSub->name, pPortName );
			pTSFifo = new TSFifo( pPortName, pSub );
		}
		if ( pTSFifo->m_pSubRecord != pSub )
		{
			printf( "Error %s: Unable to register as TSFifo port %s already registered to %s\n",
					pSub->name, pPortName, pTSFifo->m_pSubRecord->name );
			delete pTSFifo;
			return -1;
		}
		if ( pTSFifo->RegisterTimeStampSource() != asynSuccess )
		{
			printf( "Error %s: Unable to register timeStampSource for port %s\n",
					pSub->name, pPortName );
			delete pTSFifo;
			return -1;
		}
		printf( "%s: Successfully registered timeStampSource for port %s\n",
				pSub->name, pPortName );
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

#if 0
	double	*	pDblVal	= static_cast<double *>( pSub->d );
	if ( pDblVal != NULL )
		pTSFifo->m_delay		= *pDblVal;
#else
	pIntVal	= static_cast<epicsInt32 *>( pSub->d );
	if ( pIntVal != NULL )
		pTSFifo->m_delay		= *pIntVal;
#endif

	pIntVal	= static_cast<epicsInt32 *>( pSub->e );
	if ( pIntVal != NULL )
	{
		TSFifo::TSPolicy	tsPolicy = static_cast<TSFifo::TSPolicy>(*pIntVal);
		pTSFifo->SetTimeStampPolicy( tsPolicy );
	}

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
	if ( args[0].sval == 0 )
	{
		printf( "Usage: ShowTSFifo portName level\n" );
		return;
	}

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
