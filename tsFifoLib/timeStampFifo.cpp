#include <iostream>
#include <iomanip>
#include <math.h>
#include <string>
#include <string.h>
#include <stdlib.h>
#include <sstream>

#include <iocsh.h>
#include <registryFunction.h>
#include <epicsExport.h>
// #include <epicsThread.h>
#include <dbFldTypes.h>
#include <aSubRecord.h>
#include <dbAddr.h>
#include <dbAccess.h>
#include <dbScan.h>
#include <recGbl.h>

#include "asynDriver.h"
#include "timingFifoApi.h"
#include "timeStampFifo.h"
#include "HiResTime.h"

extern double camera_ts;

using namespace		std;

int					DEBUG_TS_FIFO	= 1;

#ifndef NULL
#define NULL    0
#endif
#define	TIMESTAMP_NSEC_FID_MASK	0x1FFFF
/*
 * What is the largest difference between an internal an EVR/TPR timestamp
 * that we will call a hit?
 */
#define MAX_INT_DELTA_MS     3.0
/*
 * What is the largest differnce between an internal an EVR/TPR timestamp
 * that will cause us to give up?
 */
#define MAX_DELTA_MS        30.0

using namespace		std;

/// Static TSFifo map by port name
map<string, TSFifo *>   TSFifo::ms_TSFifoMap;

extern "C" {
static const char *ts_time_str()
{
    epicsTimeStamp now;
    static char nowText[40];
    size_t rtn;

    rtn = epicsTimeGetCurrent(&now);
    if(rtn)
        return "";
    nowText[0] = 0;
    epicsTimeToStrftime(nowText,sizeof(nowText),
			"%Y/%m/%d %H:%M:%S.%03f ",&now);
    return nowText;
}
};

///	timingGetFiducialForTimeStamp returns the 64 bit fiducial that corresponds to the specified timestamp.
///	If the timing module cannot determine the correct fiducial, it returns TIMING_PULSEID_INVALID.
///	TODO: For LCLS2 mode we should combine sec and nsec to get the 64 bit fiducial.
//  Caveat: We're only using this function for diag msgs which will be mostly useful for LCLS1 timing,
//  so it's not an urgent TODO.
extern epicsUInt64 timingGetFiducialForTimeStamp( epicsTimeStamp timeStamp )
{       
	epicsUInt32     fiducial = timeStamp.nsec & TIMESTAMP_NSEC_FID_MASK;
	if ( fiducial == TIMESTAMP_NSEC_FID_MASK )
		return TIMING_PULSEID_INVALID;
	else
		return (TimingPulseId) fiducial;
}

#define LCLS1_FID_MAX        0x1ffe0
#define LCLS1_FID_ROLL_LO    0x00200
#define LCLS1_FID_ROLL_HI    (LCLS1_FID_MAX-LCLS1_FID_ROLL_LO)
#define LCLS1_FID_ROLL(a,b)  ((b) < LCLS1_FID_ROLL_LO && (a) > LCLS1_FID_ROLL_HI)
#define LCLS1_FID_GT(a,b)    (LCLS1_FID_ROLL(b, a) || ((a) > (b) && !LCLS1_FID_ROLL(a, b)))
#define LCLS1_FID_DIFF(a,b)  ((LCLS1_FID_ROLL(b, a) ? LCLS1_FID_MAX : 0) + (int)(a) - (int)(b) - (LCLS1_FID_ROLL(a, b) ? LCLS1_FID_MAX : 0))

///	fiducialDiff returns the 64 bit delta from fid1 - fid2
extern epicsInt64 fiducialDiff( TimingPulseId fid1, TimingPulseId fid2 )
{
	epicsInt64 	fidDiff;
#ifdef HAVE_LCLS2MODE
	if ( m_Lcls2Mode )
		fidDiff = fid1 - fid2;
	else
#endif // HAVE_LCLS2MODE
		fidDiff = LCLS1_FID_DIFF(fid1, fid2);
	return fidDiff;
}

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
	if ( pTSFifo == NULL )
	{
		printf( "Error %s: userPvt param is NULL!\n", functionName );
		return;
	}

	// Get the timestamp
	status = pTSFifo->GetTimeStamp( pTimeStamp );
	if ( status != asynSuccess )
	{
		// Defaults to best available timestamp w/ TIMING_PULSEID_INVALID on error
		epicsTimeGetCurrent( pTimeStamp );
		pTimeStamp->nsec |= TIMESTAMP_NSEC_FID_MASK;
		if ( (DEBUG_TS_FIFO & 8) && (DEBUG_TS_FIFO & 2) )
			printf( "Error %s: GetTimeStamp error %d for port %s\n",
					functionName, status, pTSFifo->GetPortName() );
	}
	return;
}


/// Constructor for TSFifo
///	TODO: Add m_Lcls2Mode argument to constructor.  Or, grab it via aSubRecord function TSFifo_Process.
TSFifo::TSFifo(
	const char	*	pPortName,
	aSubRecord	*	pSubRecord,
	TSPolicy		tsPolicy	)
	:	m_eventCode(	0				),
		m_genCount(		0				),
		m_genPrior(		0				),
		m_delay(		0.0				),
		m_expDelay(		0.0				),
		m_synced(		false			),
		m_diffVsExp(	0.0				),
		m_diffVsExpMin(	0.0				),
		m_diffVsExpMax(	0.0				),
		m_diffVsInt(	0.0				),
		m_diffVsIntMin(	0.0				),
		m_diffVsIntMax(	0.0				),
		m_diffVsIntAvg(	0.0				),
                m_intreq(       TS_NONE                         ),
                m_intreq_in(    true                            ),
		m_pSubRecord(	pSubRecord		),
                m_delta(        0.0                             ),
		m_portName(		pPortName		),
		m_idx(			0LL				),
		m_idxIncr(		TS_INDEX_INIT	),
		m_fidPrior(		TIMING_PULSEID_INVALID	),
		m_fidDiffPrior(	0				),
		m_syncCount(	0				),
		m_syncCountMin(	1				),
		m_tscNow(		0LL				),
		m_fifoDelay(	0.0				),
		m_fidFifo(		TIMING_PULSEID_INVALID	),
		m_TSPolicy(		tsPolicy		),
		m_TSPolicyPrior(	TS_TOD	                ),
                m_last_camera_ts(       0.0                     ),
                m_last_idx(       0LL                           ),
                m_last_ec(              0                       ),
		m_TSLock(		0				)
{
	m_TSLock	= epicsMutexCreate( );
	if ( m_TSLock )
		AddTSFifo( this );
	else
		printf( "TSFifo: Unable to register TSFifo due to epicsMutexCreate error!\n" );
}

/// Destructor
TSFifo::~TSFifo( )
{
	if ( m_TSLock )
	{
		epicsMutexLock( m_TSLock );
		DelTSFifo( this );
		epicsMutexDestroy( m_TSLock );
		m_TSLock = 0;
	}
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
	double	secPerTick	= HiResTicksToSeconds( 1LL );
	if ( DEBUG_TS_FIFO >= 2 )
		printf( "TimeStampSource: Registered %s.  TimerResolution = %.3esec per tick\n",
				m_portName.c_str(), secPerTick );
	return asynSuccess;
}

enum SyncType		{ FIFO_NEXT, FIFO_DLY, FID_DIFF, TOO_LATE, FAILED };
const char * SyncTypeToStr( SyncType tySync )
{
	const char	*	pStr	= "Invalid";
	switch ( tySync )
	{
	case FIFO_NEXT:		pStr	= "FIFO_NEXT";	break;
	case FIFO_DLY:		pStr	= "FIFO_DLY";	break;
	case FID_DIFF:		pStr	= "FID_DIFF";	break;
	case TOO_LATE:		pStr	= "TOO_LATE";	break;
	case FAILED:		pStr	= "FAILED";		break;
	}
	return pStr;
}

/// GetTimeStamp:  Get the timestamp for the configured event code.
/// In LCLS1 mode, a pulse id is encoded into the least significant 17 bits of the nsec timestamp
///	field, as per SLAC convention for EVR timestamps.
/// The pulse id is set to 0x1FFFF if the timeStampFifo status is unsynced.
///	Behavior depends on the value of m_TSPolicy:
///   TS_LAST_EC- Most recent timestamp for the specified event code, no matter how old
///   TS_SYNCED - If unsynced, no timestamp is provided and GetTimeStamp returns -1.
///   TS_TOD    - Provides a synced, pulse id'd timestamp for the specified event code
///				  if available.  If not, it provides the current time w/ the most recent
///				  fiducial pulse id.
int TSFifo::GetTimeStamp(
	epicsTimeStamp	*	pTimeStampRet )
{
	const char		*	functionName	= "TSFifo::GetTimeStamp";
	int					evrTimeStatus	= 0;
	epicsTimeStamp		curTimeStamp;
	bool				fFirstUpdate	= true;
	unsigned int		nStepBacks		= 0;
	enum SyncType		tySync			= FAILED;

	if ( pTimeStampRet == NULL )
		return -1;

	// Update the 64bit timestamp counter
	m_tscNow	= GetHiResTicks();

	//	Lock mutex
	epicsMutexLock( m_TSLock );

	// Fetch the most recent timestamp for this event code
	evrTimeStatus	= timingGetEventTimeStamp( &curTimeStamp, m_eventCode );

	// Get the last 360hz Fiducial seen by the driver
	epicsUInt32	fid360	= timingGetLastFiducial();

	if ( m_TSPolicy == TS_INTERNAL )
	{
	    double deltaPrev = m_delta;
	    if (m_TSPolicyPrior != m_TSPolicy) { /* Just starting!! */
		m_diffVsInt	= 0.0;
		m_diffVsIntMin	= 1.0;
		m_diffVsIntMax	= -1.0;
		m_diffVsIntAvg	= 0.0;
		m_delta         = 0.0;
		m_synced = false;
		m_idxIncr = TS_INDEX_INIT;
		m_intreq_in = true;
		if ( DEBUG_TS_FIFO >= 1 )
		    printf( "%s%s: INTERNAL, initializing!\n", ts_time_str(), functionName);
		/* For now, fall through and default to LAST_EC. */
	    } else if (camera_ts == -1.0 || 
		       camera_ts < m_last_camera_ts ||
		       (m_synced && camera_ts > m_last_camera_ts + 120.)) {
		/* If the camera timestamps fail, go unsynched. */
		if ( DEBUG_TS_FIFO >= 1 )
		    printf( "%s%s: INTERNAL, camera TS failed, not synchronized! (ts=%g)\n",
			    ts_time_str(), functionName, camera_ts);
		m_delta = 0.0;
		m_synced = false;
		m_idxIncr = TS_INDEX_INIT;
	    } else {
		double new_ts, new_diff, new_diff_abs, last_diff = 0.0;
		int cnt = 0, dir = 1;
		uint64_t idx_new;
#define LOGMAX 100
		struct {
		    epicsTimeStamp s;
		    uint64_t       idx;
		    float          diff;
		} log[LOGMAX];
		int logcnt = 0;

		switch (m_intreq) {
		case TS_SET:
		    if (timingFifoRead( m_eventCode, TS_INDEX_INIT, 
					&m_last_idx, &m_fifoInfo )) {
			m_delta = 0.0;
			break; /* No TS?!? */
		    }
		    if ( DEBUG_TS_FIFO >= 5 )
			printf( "%s%s: INTERNAL, timingFifoRead %d.%09d (fid=0x%x, idx=%lu)!\n",
				ts_time_str(), functionName,
				m_fifoInfo.fifo_time.secPastEpoch,
				m_fifoInfo.fifo_time.nsec,
				m_fifoInfo.fifo_time.nsec & 0x1ffff,
				m_last_idx);
		    m_delta = ((double)m_fifoInfo.fifo_time.secPastEpoch + 
			       (double)m_fifoInfo.fifo_time.nsec / 1.e9) - camera_ts;
		    if ( DEBUG_TS_FIFO >= 1 )
			printf( "%s%s: INTERNAL, set delta to %.19g, idx to %lu\n",
				ts_time_str(), functionName, m_delta, m_last_idx);
		    m_last_ec = m_eventCode;
		    m_synced = true;
		    m_intreq = TS_NONE;
		    m_intreq_in = false;
		    *pTimeStampRet = m_fifoInfo.fifo_time;
		    m_last_camera_ts = camera_ts;
		    /* Since we are resynching, clear the stats! */
		    m_diffVsInt	= 0.0;
		    m_diffVsIntMin	= 1.0;
		    m_diffVsIntMax	= -1.0;
		    m_diffVsIntAvg	= 0.0;
		    epicsMutexUnlock( m_TSLock );
		    if ( m_pSubRecord != NULL )	{
			dbCommon *pDbCommon = reinterpret_cast<dbCommon *>( m_pSubRecord );
			scanOnce( pDbCommon );
		    }
		    return 0;
		case TS_TWEAK_FWD:
		case TS_TWEAK_REV:
		    if (m_delta == 0.0)
			break;
		    // m_last_camera_ts matches the current m_idx for m_last_ec!  So
		    // just move one and recalculate!
		    if (timingFifoRead( m_last_ec, (m_intreq == TS_TWEAK_FWD) ? 1 : -1,
					&m_last_idx, &m_fifoInfo )) {
			if ( DEBUG_TS_FIFO >= 1 )
			    printf( "%s%s: INTERNAL, tweak %s failed to read timestamp fifo!\n",
				    ts_time_str(), functionName,
				    (m_intreq == TS_TWEAK_FWD) ? "forward" : "backward");
			m_delta = 0.0; /* No TS?!? */
			break; 
		    }
		    m_delta = ((double)m_fifoInfo.fifo_time.secPastEpoch + 
			       (double)m_fifoInfo.fifo_time.nsec / 1.e9) - m_last_camera_ts;
		    if ( DEBUG_TS_FIFO >= 1 )
			printf( "%s%s: INTERNAL, tweak delta %s to %.19g, idx to %lu\n",
				ts_time_str(), functionName,
				(m_intreq == TS_TWEAK_FWD) ? "forward" : "backward",
				m_delta, m_last_idx);
		    m_intreq = TS_NONE;
		    m_intreq_in = false;
		    // Fall through!
		case TS_NONE:
		    if (m_delta == 0.0)
			break; /* Fall through to LAST_EC. */
		    new_ts = m_delta + camera_ts;
		    if ( DEBUG_TS_FIFO >= 5 )
			printf( "%s%s: INTERNAL, %.19g --> %.19g\n", 
				ts_time_str(), functionName, camera_ts, new_ts);
		    /*
		     * If the event code has changed, we need to start over at the end.
		     * If it hasn't, just go forward one.
		     */
		    if (m_eventCode == m_last_ec) {
			idx_new = m_last_idx;
			dir = 1;
		    } else {
			idx_new = 0;
			dir = TS_INDEX_INIT;
			m_last_ec = m_eventCode;
		    }
		    while (1) {
			cnt++;
			if (timingFifoRead( m_eventCode, dir, &idx_new, &m_fifoInfo )) {
			    if ( DEBUG_TS_FIFO >= 1 )
				printf( "%s%s: INTERNAL, timingFifoRead failed!\n",
					ts_time_str(), functionName);
			    m_synced = false;
			    break;
			}
			if ( DEBUG_TS_FIFO >= 5 )
				printf( "%s%s: INTERNAL, timingFifoRead %d.%09d (fid=0x%x, idx=%lu)!\n",
					ts_time_str(), functionName,
					m_fifoInfo.fifo_time.secPastEpoch,
					m_fifoInfo.fifo_time.nsec,
					m_fifoInfo.fifo_time.nsec & 0x1ffff,
					idx_new);
		        /*
			 * A note on this... if new_diff is negative, then we need to make 
			 * it larger... so the queue time needs to become bigger! --> dir = 1.
			 * Similarly, if new_diff is positive, then we need to make it smaller,
			 * so dir = -1 is needed!
			 */
			new_diff = (((double)m_fifoInfo.fifo_time.secPastEpoch + 
				     (double)m_fifoInfo.fifo_time.nsec / 1.e9) - new_ts) * 1000.0;
			new_diff_abs = fabs(new_diff);
			if (logcnt < LOGMAX) {
			    log[logcnt].s  = m_fifoInfo.fifo_time;
			    log[logcnt].idx = idx_new;
			    log[logcnt].diff = new_diff;
			    logcnt++;
			}
			if (new_diff_abs < MAX_INT_DELTA_MS) {
			    // Got it!
			    if ( DEBUG_TS_FIFO >= 5 )
				printf( "%s%s: INTERNAL, found TS after increment %d, diff = %g\n",
					ts_time_str(), functionName, cnt, new_diff);
			    m_synced = true;
			    m_diffVsInt = new_diff;
			    m_diffVsIntAvg = 0.8 * m_diffVsIntAvg + 0.2 * new_diff;
			    if (new_diff < m_diffVsIntMin)
				m_diffVsIntMin = new_diff;
			    if (new_diff > m_diffVsIntMax)
				m_diffVsIntMax = new_diff;
			    // Correct delta if drifting off.
			    if (m_diffVsIntAvg > MAX_INT_DELTA_MS/2.) {
				if ( DEBUG_TS_FIFO >= 2 )
				    printf( "%s%s: INTERNAL, increasing delta by %gms\n",
					    ts_time_str(), functionName, MAX_INT_DELTA_MS/2.);
				m_delta += MAX_INT_DELTA_MS/2000.;
				m_diffVsInt -= 1;
				m_diffVsIntAvg -= 1;
			    } else if (m_diffVsIntAvg < -MAX_INT_DELTA_MS/2.) {
				if ( DEBUG_TS_FIFO >= 2 )
				    printf( "%s%s: INTERNAL, decreasing delta by %gms\n",
					    ts_time_str(), functionName, MAX_INT_DELTA_MS/2.);
				m_delta -= MAX_INT_DELTA_MS/2000.;
				m_diffVsInt += 1;
				m_diffVsIntAvg += 1;
			    }
			    *pTimeStampRet = m_fifoInfo.fifo_time;
			    m_last_camera_ts = camera_ts;
			    m_last_idx = idx_new;
			    epicsMutexUnlock( m_TSLock );
			    if ( m_pSubRecord != NULL )	{
				dbCommon *pDbCommon = reinterpret_cast<dbCommon *>( m_pSubRecord );
				scanOnce( pDbCommon );
			    }
			    return 0;
			}
			if (last_diff != 0.0 && ((new_diff < 0) != (last_diff < 0))) {
			    /*
			     * If this isn't our first point, and we've crossed zero,
			     * give up!
			     */
			    if ( DEBUG_TS_FIFO >= 5 )
				printf( "%s%s: INTERNAL, crossed zero, giving up!\n",
					ts_time_str(), functionName);
			    m_synced = false;
			    break;
			}
			dir = (new_diff < 0) ? 1 : -1;
			last_diff = new_diff;
			if (DEBUG_TS_FIFO >= 5)
			    printf("%s%s: INTERNAL, diff = %g, cnt = %d, moving on.\n",
				   ts_time_str(), functionName, new_diff, cnt);
		    }
		    /*
		     * We couldn't match the timestamp.  Turn off internal timestamping
		     * and just use LAST_EC.
		     */
		    if ( DEBUG_TS_FIFO >= 1 )
			printf( "%s%s: INTERNAL, no ts found, unsynchronizing)!\n", 
				ts_time_str(), functionName );
		    if ( DEBUG_TS_FIFO >= 2 ) {
			int i;
			const char *now = ts_time_str();
			printf("%s%s: INTERNAL, last match, idx=%lu, camera_ts=%g\n",
			       now, functionName, m_last_idx, m_last_camera_ts);
			printf( "%s%s: INTERNAL, %g --> %.19g\n", 
				now, functionName, camera_ts, new_ts);
			for (i = 0; i < logcnt; i++) {
			    printf( "%s%s: INTERNAL, timingFifoRead %d.%09d (fid=0x%x, idx=%lu), diff=%g\n",
				    now, functionName,
				    log[i].s.secPastEpoch,
				    log[i].s.nsec,
				    log[i].s.nsec & 0x1ffff,
				    log[i].idx, 
				    log[i].diff); 
			}
		    }
		    m_delta = 0.0;
		    break;
		}
	    }
	    if (m_delta != deltaPrev && m_pSubRecord != NULL) {
		// We're falling through, but something has changed, so report it!
		dbCommon *pDbCommon = reinterpret_cast<dbCommon *>( m_pSubRecord );
		scanOnce( pDbCommon );
	    }
	    if (m_intreq != TS_NONE) { // Ack the request.
		m_intreq_in = true;
		m_intreq = TS_NONE;
	    }
        }

	m_TSPolicyPrior = m_TSPolicy;
	m_synced	= false;

	if ( m_TSPolicy == TS_LAST_EC || m_TSPolicy == TS_INTERNAL )
	{
		// If timingGetEventTimeStamp is happy assume we're synced in LAST_EC.
	        // If this is falling through from INTERNAL, we are *not* synced though!
		if ( evrTimeStatus == 0 && m_TSPolicy == TS_LAST_EC )
			m_synced = true;

		*pTimeStampRet = curTimeStamp;
		evrTimeStatus = UpdateFifoInfo( fFirstUpdate );
		fFirstUpdate = false;
		epicsMutexUnlock( m_TSLock );

		if ( DEBUG_TS_FIFO >= 5 )
			printf( "%s%s: LAST_EC, expectedDelay=%.2fms, fifoDelay=%.2fms, fid 0x%llX\n",
				ts_time_str(), functionName,
					m_expDelay * 1000, m_fifoDelay * 1000, timingGetFiducialForTimeStamp(curTimeStamp) );
		return evrTimeStatus;
	}

	if ( m_TSPolicy == TS_TOD )
	{
		// Just get the latest system timestamp
		epicsTimeStamp		todTimeStamp;
		evrTimeStatus	= epicsTimeGetCurrent( &todTimeStamp ); 
		*pTimeStampRet	= todTimeStamp;

		if ( DEBUG_TS_FIFO >= 5 )
		{
			char		acBuff[40];
			epicsTimeToStrftime( acBuff, 40, "%H:%M:%S.%04f", &todTimeStamp );
			printf( "%s%s: TOD, %s\n", ts_time_str(), functionName, acBuff );
		}
		return 0;
	}

	bool	fifoReset	= false;
	if ( m_idxIncr == TS_INDEX_INIT )
		fifoReset	= true;

	// First time or unsynced, m_idxIncr is TS_INDEX_INIT, which
	// just gets the most recent FIFO timestamp for that eventCode
	evrTimeStatus = UpdateFifoInfo( fFirstUpdate );
	fFirstUpdate = false;
	if ( evrTimeStatus == 0 && m_diffVsExp > 60e-3 )
	{
		if ( m_idxIncr != TS_INDEX_INIT )
		{
			if ( DEBUG_TS_FIFO > 5 )
				printf( "%s%s: Reject FIFO, expectedDelay=%.2fms, fifoDelay=%.2fms, diffVsExp=%.2fms, idxIncr=%d\n",
					ts_time_str(), functionName, m_expDelay * 1000,
					m_fifoDelay * 1000, m_diffVsExp * 1000, m_idxIncr );

			// This FIFO entry is stale, reset and get the most recent
			fifoReset	  = true;
			m_idxIncr     = TS_INDEX_INIT;
			evrTimeStatus = UpdateFifoInfo( fFirstUpdate );
			fFirstUpdate = false;
		}
		else
		{
			if ( DEBUG_TS_FIFO >= 5 )
				printf( "%s%s: Stale  FIFO, expectedDelay=%.2fms, fifoDelay=%.2fms, diffVsExp=%.2fms\n",
					ts_time_str(), functionName, m_expDelay * 1000, 
					m_fifoDelay * 1000, m_diffVsExp * 1000 );
		}
	}
	if ( evrTimeStatus != 0 )
	{
		// Nothing available, reset the FIFO increment and give up
		m_idxIncr     = TS_INDEX_INIT;
		epicsMutexUnlock( m_TSLock );
		if ( DEBUG_TS_FIFO >= 5 )
		{
			printf( "%s GetTimeStamp error fetching fifo info for eventCode %d, incr %d: evrTimeStatus=%d, fidFifo=%llu\n",
				ts_time_str(), m_eventCode, m_idxIncr, evrTimeStatus,
				timingGetFiducialForTimeStamp( m_fifoInfo.fifo_time ) );
		}
		return evrTimeStatus;
	}

	// Good timestamp from FIFO
	// Set m_idxIncr to advance one next time
	m_idxIncr		= 1;

	// Compare the fiducial for this FIFO timestamp with the target
	// Compute the target error and delta vs the prior fiducial
	// TODO: This is the only place where LCLS2 mode won't work
	// The concept behind fidDiff relies on the regular increment
	// by 1 of the LCLS1 fiducial at 360hz.
	// fidDiff is only used if the expected delay is outside the
	// tolerance.  In which case the code would look at fidDiff
	// to see if it was the expected value.
	// i.e 3 for 120hz, 6 for 60hz, etc
	// That test is no longer used for gigE cameras, even in LCLS1
	// mode as it doesn't handle missing frames well.
	// It's not clear that it ever hits even in LCLS1 mode.
	int64_t		fidDiff	 = TIMING_PULSEID_INVALID;
	if ( m_fidFifo != TIMING_PULSEID_INVALID )
	{
		if ( m_fidPrior != TIMING_PULSEID_INVALID )
			fidDiff	= fiducialDiff( m_fidFifo, m_fidPrior );
	}

	if ( DEBUG_TS_FIFO >= 5 )
	{
		if ( m_fidFifo == TIMING_PULSEID_INVALID )
			printf( "%s%s: %s Error FIFO, expectedDelay=%.2fms, fifoDelay=%.2fms, fidFifo 0x%lX\n",
				ts_time_str(), functionName, ( fifoReset ? "Reset" : "Next " ),
				m_expDelay * 1000, m_fifoDelay * 1000, m_fidFifo );
		else
			printf( "%s%s: %s  FIFO, expectedDelay=%.2fms, fifoDelay=%.2fms\n",
				ts_time_str(), functionName, ( fifoReset ? "Reset" : "Next " ),
				m_expDelay * 1000, m_fifoDelay * 1000 );
	}

	// Did we hit our target pulse?
	// Original test:
	// Allow -2ms for sloppy estimated delay and +7ms for late pickup
	//if ( -2e-3 < m_diffVsExp && m_diffVsExp <= 7e-3 )
	// New test is proportional to allow for variations in long transmit
	// time for gigE cameras.
	// Allow 40% early for sloppy estimated delay and 80% late
	double	diffVsExpPercent = m_diffVsExp * 100.0 / m_expDelay; 
	if ( -40.0 < diffVsExpPercent && diffVsExpPercent <= 80.0 )
	{
		// We're synced!
		m_synced	= true;
		m_syncCount++;
		tySync	= FIFO_NEXT;
		if( m_diffVsExpMax < m_diffVsExp )
			m_diffVsExpMax = m_diffVsExp;
		if( m_diffVsExpMin > m_diffVsExp )
			m_diffVsExpMin = m_diffVsExp;
	}
	else
	{
		// Check earlier entries in the FIFO
	        while ( m_diffVsExp <= (2*m_expDelay) && m_fifoDelay > -1e-3 )
		{
			nStepBacks++;
			m_idxIncr     = -1;
			evrTimeStatus = UpdateFifoInfo( fFirstUpdate );
			fFirstUpdate = false;
			if ( evrTimeStatus != 0 )
			{
				// FIFO is empty
				// Reset FIFO so we get the most recent entry next time
				m_idxIncr	= TS_INDEX_INIT;
				tySync		= FAILED;
				m_synced	= false;
				m_syncCount	= 0;
				break;
			}

			if ( DEBUG_TS_FIFO >= 5 )
				printf( "%s%s FIFO incr %2d: expectedDelay=%.3fms, fifoDelay=%.3fms, diffVsExp=%.3f\n",
					ts_time_str(), functionName, m_idxIncr,
					m_expDelay*1000, m_fifoDelay*1000, m_diffVsExp*1000 );

			double	diffVsExpPercent = m_diffVsExp * 100.0 / m_expDelay; 
			if ( -40.0 < diffVsExpPercent && diffVsExpPercent <= 80.0 )
			{
				// Found a match!
				tySync		= FIFO_DLY;
				m_idxIncr	= 1;
				m_syncCount	= 0;
				m_synced	= true;	// not yet?
				break;
			}
		}
	}

	// Remember prior values
	m_fidPrior		= m_fidFifo;
	m_fidDiffPrior	= fidDiff;

	// Check for a generation change
	if( m_genPrior != m_genCount )
		m_synced	= false;
	m_genPrior		= m_genCount;

	if ( !m_synced )
	{
		//	Mark unsynced and reset FIFO selector
		m_idxIncr			  = TS_INDEX_INIT;
		m_fifoTimeStamp.nsec |= TIMESTAMP_NSEC_FID_MASK;
	}

	if (	( DEBUG_TS_FIFO & 4 )
		|| (( DEBUG_TS_FIFO & 2 ) && m_synced ) )
	{
		char		acBuff[40];
		epicsTimeToStrftime( acBuff, 40, "%H:%M:%S.%04f", &m_fifoTimeStamp );
		printf( "%s%s: %-8s, %-8s, ts %s, fid 0x%llX, fidFifo 0x%lX, fid360 0x%X, fidDiff %ld, fidDiffPrior %ld\n",
			ts_time_str(), functionName,
			( m_synced ? "Synced" : "Unsynced" ),
			SyncTypeToStr( tySync ),
			acBuff, timingGetFiducialForTimeStamp(m_fifoTimeStamp), m_fidFifo, fid360,
			fidDiff, m_fidDiffPrior	);
	}
	epicsMutexUnlock( m_TSLock );

	if ( m_pSubRecord != NULL )
	{
		dbCommon	*	pDbCommon	= reinterpret_cast<dbCommon *>( m_pSubRecord );
		scanOnce( pDbCommon );
	}

	if ( m_TSPolicy == TS_SYNCED && !m_synced )
		return -1;

	// If we have a pulse ID's timestamp, return it
	if ( timingGetFiducialForTimeStamp(m_fifoTimeStamp) != TIMING_PULSEID_INVALID )
		*pTimeStampRet = m_fifoTimeStamp;
	return evrTimeStatus;
}


/// UpdateFifoInfo:  Get the latest fifoInfo for the specified increment
/// Must be called w/ m_TSLock mutex locked!
int TSFifo::UpdateFifoInfo( bool fFirstUpdate )
{
	m_fidFifo				 = TIMING_PULSEID_INVALID;
	m_fifoTimeStamp.nsec	|= TIMESTAMP_NSEC_FID_MASK;
	m_fifoDelay				 = 0;
	m_diffVsExp				 = 0;

	if ( m_idxIncr == TS_INDEX_INIT )
		m_fidPrior = TIMING_PULSEID_INVALID;

	int evrTimeStatus = timingFifoRead( m_eventCode, m_idxIncr, &m_idx, &m_fifoInfo );
	if ( evrTimeStatus != 0 )
	{
		// 5 possible failure modes for timingFifoRead()
		//	1.	Invalid event code
		//		Timestamp not updated
		//	2.	m_idxIncr was already TS_INDEX_INIT and no entries in the FIFO for this event code
		//		Timestamp not updated
		//	3.	m_idxIncr was 1 and FIFO is drained
		//		i.e. we've already requested the most recent entry
		//		Invalid event code or no entries in the FIFO for this event code
		//		Timestamp not updated
		//	4.	non-zero fifostatus in FIFO entry
		//		Timestamp updated but likely has bad fiducial ID
		//		Probably because evrTimeEventProcessing() thinks we aren't synced
		if ( DEBUG_TS_FIFO >= 5 )
		{
			printf( "UpdateFifoInfo error fetching fifo info for eventCode %d, incr %u: evrTimeStatus=%d, fidFifo=%lld\n",
					m_eventCode, m_idxIncr, evrTimeStatus,
					timingGetFiducialForTimeStamp( m_fifoInfo.fifo_time ) );
		}

		if ( m_idxIncr != TS_INDEX_INIT )
		{
			// Reset the FIFO and get the most recent entry
			m_idxIncr = TS_INDEX_INIT;
			evrTimeStatus = timingFifoRead( m_eventCode, TS_INDEX_INIT, &m_idx, &m_fifoInfo );
			if ( evrTimeStatus != 0 && ( DEBUG_TS_FIFO >= 5 ) )
			{
				printf( "UpdateFifoInfo error on reset fetch of fifo info for eventCode %d: evrTimeStatus=%d\n", m_eventCode, evrTimeStatus );
			}
		}
	}

	if ( evrTimeStatus == 0 )
	{
		// Good timestamp from FIFO
		m_fifoTimeStamp	= m_fifoInfo.fifo_time;
		m_fidFifo		= timingGetFiducialForTimeStamp( m_fifoTimeStamp );

		// Compute the delay in seconds since this m_fifoInfo event was collected
		m_fifoDelay		= HiResTicksToSeconds( m_tscNow - m_fifoInfo.fifo_tsc );
		if ( fFirstUpdate )
		{
			if( m_fifoDelayMin == 0 || m_fifoDelayMin > m_fifoDelay )
				m_fifoDelayMin = m_fifoDelay;
			if( m_fifoDelayMax < m_fifoDelay )
				m_fifoDelayMax = m_fifoDelay;
		}
		m_diffVsExp		= m_fifoDelay - m_expDelay;
		if ( DEBUG_TS_FIFO >= 7 )
		{
			t_HiResTime	tscNow	= GetHiResTicks();
			double tscDelay	= HiResTicksToSeconds( tscNow - m_tscNow );
			printf( "UpdateFifoInfo: EC=%d, incr=%u, fidFifo=%lu, m_tscNow=%llu, fifoTsc=%zd, tscDelay=%0.3f\n",
					m_eventCode, m_idxIncr, m_fidFifo, m_tscNow, m_fifoInfo.fifo_tsc, tscDelay*1000 );
		}
	}
	return evrTimeStatus;
}

void TSFifo::ResetExpectedDelay()
{
	if ( DEBUG_TS_FIFO >= 1 )
	{
		printf( "expDelay=%.2fms, earliest=expDelay%.3fms, latest=expDelay+%.3fms\n",
				m_expDelay * 1000, m_diffVsExpMin * 1000, m_diffVsExpMax * 1000 );
	}
	m_diffVsExpMin	= 0.0;
	m_diffVsExpMax	= 0.0;
	m_fifoDelayMin	= 0.0;
	m_fifoDelayMax	= 0.0;
}

epicsUInt32	TSFifo::Show( int level ) const
{
	printf( "TSFifo for port %s\n",	m_portName.c_str() );
	printf( "\tEventCode:\t%d\n",	m_eventCode );
	printf( "\tGeneration:\t%d\n",	m_genCount );
	printf( "\tExpDelay:\t%.2fms,\tearliest=%.3fms,\tlatest=%.3fms\n",
			m_expDelay * 1000, m_diffVsExpMin * 1000, m_diffVsExpMax * 1000 );
	printf( "\tTS Policy:\t%s\n",	(	m_TSPolicy == TS_LAST_EC
									?	"LAST_EC"
									:	(	m_TSPolicy == TS_TOD
										?	"TOD" : "SYNCED" ) ) );
	printf( "\tSync Status:\t%s\n",	m_synced ? "Synced" : "Unsynced" );
	return 0;
}

void TSFifo::ListPorts()
{
	map<string, TSFifo *>::iterator   it;
	for ( it = ms_TSFifoMap.begin(); it != ms_TSFifoMap.end(); it++ )
		printf( "%s ", it->first.c_str() );
	printf( "\n" );
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
//		B:	Beam Event code for timestamp
//		C:	Generation counter for EventCode timing, should increment on any timing change
//		D:	Expected delay in seconds between the eventCode and the ts query
//		E:	TimeStamp policy
//		F:	TimeStamp FreeRun mode
//              G:      Internal TS Request
// TODO: Add Lcls2Mode
// 		H?:	LCLS2 Timing mode
// TODO: Add support for 2 event codes, Beam and Camera
//		I?:	Camera trigger Event code for synchronization
//
//	Outputs
//		A:	TSFifo Sync Status: 0 = unlocked, 1 = locked
//		B:	DiffVsExp,    ms
//		C:	DiffVsExpMin, ms
//		D:	DiffVsExpMax, ms
//		E:	ActualDelayMin, ms
//		F:	ActualDelayMax, ms
//              G:      Internal TS Request ACK
//		H:	DiffVsInt,    ms
//		I:	DiffVsIntMin, ms
//		J:	DiffVsIntMax, ms
//		K:	DiffVsIntAvg, ms
//
extern "C" long TSFifo_Process( aSubRecord	*	pSub	)
{
	TSFifo		*   pTSFifo	= NULL;
	int				status	= 0;
	bool			fTimeStampCriteriaChanged = false;

	if ( DEBUG_TS_FIFO & 8 )
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
			if ( DEBUG_TS_FIFO & 2 )
				printf( "%s: Port name not available yet. Still %s\n", pSub->name, pPortName );
			return -1;
		}

		if ( DEBUG_TS_FIFO )
			printf( "%s: Attempting to register port name %s\n", pSub->name, pPortName );

		pTSFifo		= TSFifo::FindByPortName( pPortName );
		if ( pTSFifo == NULL )
		{
			if ( DEBUG_TS_FIFO )
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
	if (	pIntVal != NULL
		&&	*pIntVal > 0	)
	{
		if( pTSFifo->m_eventCode	!= static_cast<epicsUInt32>(*pIntVal) )
		{
			pTSFifo->m_eventCode	= static_cast<epicsUInt32>(*pIntVal);
			fTimeStampCriteriaChanged = true;
		}
	}

	pIntVal	= static_cast<epicsInt32 *>( pSub->c );
	if ( pIntVal != NULL )
	{
		if( pTSFifo->m_genCount	!= static_cast<epicsUInt32>(*pIntVal) )
		{
			pTSFifo->m_genCount	= static_cast<epicsUInt32>(*pIntVal);
			fTimeStampCriteriaChanged = true;
		}
	}

	double	*	pDblVal	= static_cast<double *>( pSub->d );
	if ( pDblVal != NULL )
	{	// Fetch the expected delay in sec between the trigger and the timestamp update
		pTSFifo->m_delay		= *pDblVal;
		if ( pTSFifo->m_expDelay != pTSFifo->m_delay )
		{
			pTSFifo->m_expDelay		= pTSFifo->m_delay;
			fTimeStampCriteriaChanged = true;
		}
	}

	// First see if we're in FreeRun mode
	pIntVal	= static_cast<epicsInt32 *>( pSub->f );
	if ( pIntVal != NULL  && *pIntVal == 1 )
	{
		// Always use TOD in FreeRun mode
		pTSFifo->SetTimeStampPolicy( TSFifo::TS_TOD );
	}
	else	// else follow selected TsPolicy
	{
		pIntVal	= static_cast<epicsInt32 *>( pSub->e );
		if ( pIntVal != NULL )
		{
			TSFifo::TSPolicy	tsPolicy = static_cast<TSFifo::TSPolicy>(*pIntVal);
			if ( pTSFifo->GetTimeStampPolicy() != tsPolicy )
			{
				pTSFifo->SetTimeStampPolicy( tsPolicy );
				fTimeStampCriteriaChanged = true;
			}
		}
	}

	pIntVal	= static_cast<epicsInt32 *>( pSub->g );
	if ( pIntVal != NULL && pTSFifo->m_intreq_in) {
	    pTSFifo->m_intreq = (TSFifo::TSIntReq) *pIntVal;
	}

	if ( fTimeStampCriteriaChanged )
		pTSFifo->ResetExpectedDelay();

	// Update outputs
	pIntVal	= static_cast<epicsInt32 *>( pSub->vala );
	if ( pIntVal != NULL )
		*pIntVal	= pTSFifo->m_synced;

	pDblVal	= static_cast<double *>( pSub->valb );
	if ( pDblVal != NULL )
		*pDblVal	= pTSFifo->m_diffVsExp * 1000;

	pDblVal	= static_cast<double *>( pSub->valc );
	if ( pDblVal != NULL )
		*pDblVal	= pTSFifo->m_diffVsExpMin * 1000;

	pDblVal	= static_cast<double *>( pSub->vald );
	if ( pDblVal != NULL )
		*pDblVal	= pTSFifo->m_diffVsExpMax * 1000;

	pDblVal	= static_cast<double *>( pSub->vale );
	if ( pDblVal != NULL )
		*pDblVal	= pTSFifo->m_fifoDelayMin;

	pDblVal	= static_cast<double *>( pSub->valf );
	if ( pDblVal != NULL )
		*pDblVal	= pTSFifo->m_fifoDelayMax;

	pIntVal	= static_cast<epicsInt32 *>( pSub->valg );
	if ( pIntVal != NULL  && !pTSFifo->m_intreq_in) {
		*pIntVal	= pTSFifo->m_intreq;
		pTSFifo->m_intreq_in = true;
	}

	pDblVal	= static_cast<double *>( pSub->valh );
	if ( pDblVal != NULL )
		*pDblVal	= pTSFifo->m_diffVsInt;

	pDblVal	= static_cast<double *>( pSub->vali );
	if ( pDblVal != NULL )
		*pDblVal	= pTSFifo->m_diffVsIntMin;

	pDblVal	= static_cast<double *>( pSub->valj );
	if ( pDblVal != NULL )
		*pDblVal	= pTSFifo->m_diffVsIntMax;

	pDblVal	= static_cast<double *>( pSub->valk );
	if ( pDblVal != NULL )
		*pDblVal	= pTSFifo->m_diffVsIntAvg;

	pDblVal	= static_cast<double *>( pSub->vall );
	if ( pDblVal != NULL )
	        *pDblVal	= pTSFifo->m_delta;
	return status;
}


// Register aSub functions
extern "C"
{
epicsRegisterFunction(	TSFifo_Init		);
epicsRegisterFunction(	TSFifo_Process	);
epicsRegisterFunction(	TimeStampFifo	);
epicsExportAddress( int, DEBUG_TS_FIFO	);
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
	{
		printf( "Error: Unable to find TSFifo %s\n", args[0].sval );
		printf( "Available TSFifo Ports are:\n" );
		TSFifo::ListPorts();
	}
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
