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
    /// Constructor
    TSFifo(	const char			*	pPortName,
			struct	aSubRecord	*	m_pSubRecord	);

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

private:    //  Private class variables

	static  std::map< std::string, TSFifo *>   ms_TSFifoMap;
};


#endif  //  TSFIFO_H
