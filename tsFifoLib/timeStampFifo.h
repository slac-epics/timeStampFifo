#ifndef TSFIFO_H
#define TSFIFO_H

#include <map>
#include "evrTime.h"
#include "HiResTime.h"

///
/// Header file for interface between EPICS and the software used
/// to provide timestamps using the MRF EVR FIFO
///

///
/// TimeStampFifo status error codes
/// Add more as needed
///
#define TSFifo_STS_OK                 0
#define TSFifo_STS_INVALID_DATA       1

extern "C" const char	*	TSFifo_StatusToString( epicsUInt32	status	);

class   TSFifo;
struct	aSubRecord;

///
/// TSFifo is the primary data structure used to pass
/// data to and from TimeStamp operations
/// Structure members with "PV=" in the comment are obtained via EPICS PV's
///
class  TSFifo
{
public:
	/// Default timestamp policy is to provide the most recent timestamp available
	/// for the specified event code, TS_LAST_EC.  A pulse id is encoded into
	/// the least significant 17 bits of the nsec timestamp field, as per
	/// SLAC convention for EVR timestamps.   The pulse id is set to 0x1FFFF
	/// if the timeStampFifo status is unsynced.
	///   TS_LAST_EC- Most recent timestamp for the specified event code, no matter how old
	///   TS_SYNCED - If unsynced, no timestamp is provided and GetTimeStamp returns -1.
	///   TS_TOD    - Provides a synced, pulse id'd timestamp for the specified event code
	///				  if available.  If not, it provides the current time w/ the most recent
	///				  fiducial pulse id.
	enum TSPolicy	{ TS_LAST_EC = 0, TS_SYNCED = 1, TS_TOD = 2 };

    /// Constructor
    TSFifo(	const char			*	pPortName,
			struct	aSubRecord	*	pSubRecord,
			TSPolicy				tsPolicy = TS_LAST_EC );

    /// Destructor
    virtual ~TSFifo( );

	/// GetTimeStamp
	/// Synchronize w/ the timestamp FIFO and return the timestamp
	/// Returns: 0 on success
	/// On error, returns -1 and sets pTimeStampRet to the current system
	/// clock timestamp w/ the fiducial pulsid set to invalid
	int	GetTimeStamp(	epicsTimeStamp		*	pTimeStampRet );

	/// Return the current TimeStamp policy
	TSPolicy	GetTimeStampPolicy( ) const
	{
		return m_TSPolicy;
	}

	/// Set the TimeStamp policy
	void	SetTimeStampPolicy( TSPolicy	tsPolicy )
	{
		m_TSPolicy = tsPolicy;
	}

	/// ResetExpectedDelay()
	/// Resets Expected delay values for diagnostic tracking
	/// Auto-Resets on changes to timeStamp criteria
	void	ResetExpectedDelay();

	/// Show()
	/// Display pertinent TSFifo info on stdout
	epicsUInt32	Show( int level ) const;

	/// RegisterTimeStampSource()
	asynStatus RegisterTimeStampSource( );

	const char *	GetPortName( ) const
	{
		return m_portName.c_str();
	}
public:		//  Public class functions
	static	TSFifo	*	FindByPortName( const std::string & portName );
	
	static	void		ListPorts( );

private:	//  Private member functions
	int		UpdateFifoInfo( );

private:	//  Private class functions
	static	void		AddTSFifo( TSFifo * );
	static	void		DelTSFifo( TSFifo * );

public:		//  Public input member variables
	//
    //  aSub "C" function inputs
	//
    epicsUInt32				m_eventCode;	/// m_eventCode: Event code for timestamps
    epicsUInt32				m_genCount;		/// m_genCount: Increments each time EVR settings are tweaked
    epicsUInt32				m_genPrior;		/// m_genPrior: prior m_genCount
	double					m_delay;		/// m_delay:	Expected delay since event code (fid)
	double					m_expDelay;		/// m_expDelay:	Expected delay since event code (sec)

	//
	//	aSub "C" function outputs
	//
	bool					m_synced;		/// m_synced: True if synced
	double					m_diffVsExp;	/// Diff vs expectedDelay (ms)
	double					m_diffVsExpMin;	/// Minimum Diff vs expectedDelay (ms)
	double					m_diffVsExpMax;	/// Maximum Diff vs expectedDelay (ms)

	struct	aSubRecord	*	m_pSubRecord;

private:	//  Private member variables
	std::string				m_portName;
	unsigned long long		m_idx;
	unsigned int			m_idxIncr;
	int						m_fidPrior;
	int						m_fidDiffPrior;
	int						m_syncCount;
	int						m_syncCountMin;
	t_HiResTime				m_tscNow;
	evrFifoInfo				m_fifoInfo;
	epicsTimeStamp			m_fifoTimeStamp;
	double					m_fifoDelay;
	epicsUInt32				m_fidFifo;
	TSPolicy				m_TSPolicy;
	epicsMutexId			m_TSLock;

private:    //  Private class variables

	static  std::map< std::string, TSFifo *>	ms_TSFifoMap;
};


#endif  //  TSFIFO_H
