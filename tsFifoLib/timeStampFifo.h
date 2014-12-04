#ifndef TSFIFO_H
#define TSFIFO_H

#include <map>

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
	/// Default timestamp policy is to provide the best timestamp available
	/// for the specified event code, TS_EVENT.  A pulse id is encoded into
	/// the least significant 17 bits of the nsec timestamp field, as per
	/// SLAC convention for EVR timestamps.   The pulse id is set to 0x1FFFF
	/// if the timeStampFifo status is unsynced.
	///   TS_EVENT	- Most recent timestamp for the specified event code, no matter how old
	///   TS_SYNCED - If unsynced, no timestamp is provided and GetTimeStamp returns -1.
	///   TS_BEST   - Provides a synced, pulse id'd timestamp for the specified event code
	///				  if available.  If not, it provides the current time w/ the most recent
	///				  fiducial pulse id.
	enum TSPolicy	{ TS_EVENT = 0, TS_SYNCED = 1, TS_BEST = 2 };

    /// Constructor
    TSFifo(	const char			*	pPortName,
			struct	aSubRecord	*	m_pSubRecord,
			TSPolicy				tsPolicy = TS_EVENT );

    /// Destructor
    virtual ~TSFifo( )
    {
    }

	/// GetTimeStamp
	/// Synchronize w/ the timestamp FIFO and return the timestamp
	/// Returns: 0 on success
	/// On error, returns -1 and sets pTimeStampRet to the current system
	/// clock timestamp w/ the fiducial pulsid set to invalid
	int	GetTimeStamp(	epicsTimeStamp		*	pTimeStampRet );

	/// Return the current TimeStamp policy
	TSPolicy	GetTimeStampPolicy( ) const
	{
		return m_tsPolicy;
	}

	/// Set the TimeStamp policy
	void	SetTimeStampPolicy( TSPolicy	tsPolicy )
	{
		m_tsPolicy = tsPolicy;
	}

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

private:	//  Private class functions
	static	void		AddTSFifo( TSFifo * );
	static	void		DelTSFifo( TSFifo * );

public:		//  Public input member variables
	//
    //  Inputs written from aSub "C" function
	//

	/// m_eventCode: Event code for timestamps
    epicsUInt32				m_eventCode;
    epicsUInt32				m_genCount;
    epicsUInt32				m_genPrior;
	double					m_delay;

	bool					m_synced;

	struct	aSubRecord	*	m_pSubRecord;

private:	//  Private member variables
	std::string				m_portName;
	unsigned long long		m_idx;
	unsigned int			m_idxIncr;
	int						m_fidPrior;
	int						m_fidDiffPrior;
	int						m_syncCount;
	int						m_syncCountMin;
	TSPolicy				m_tsPolicy;

private:    //  Private class variables

	static  std::map< std::string, TSFifo *>   ms_TSFifoMap;
};


#endif  //  TSFIFO_H
