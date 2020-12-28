#include "Simulator.h"

#include "tinyxml.h"
//#include "windows.h"
//#include <iostream>


float SimAgent::TIME_STEP = 0.1f;

////////////////////////////////////////////////////////////////////////////
//                IMPLEMENTATION for PlaybackSimulator
////////////////////////////////////////////////////////////////////////////

PlaybackSimulator::~PlaybackSimulator() {
	for ( AgentVecItr itr = _agents.begin(); itr != _agents.end(); ++itr ) {
		delete (*itr);
	}
	if ( _classIDs != 0x0 ) delete [] _classIDs;
}

////////////////////////////////////////////////////////////////////////////

bool PlaybackSimulator::loadFromXML( const std::string & fileName ) {
	if ( fileName == "" ) return true;
	TiXmlDocument xml( fileName );
	if ( ! xml.LoadFile() ) {
		std::cerr << "Error reading simulation file: " << fileName << "." << std::endl;
		return false;
	}

	TiXmlElement* experimentNode = xml.RootElement();	
	if( ! experimentNode ) {
		std::cerr << "Root element in " << fileName << " does not exist." <<std::endl;
		return false;
	}

	if( experimentNode->ValueStr () != "Experiment" ) {
		std::cerr << "Root element of " << fileName << " is not 'Experiment'." << std::endl;
		return false;
	}

	// don't care about any of the properties of the Experiment
	TiXmlElement* child;
	for( child = experimentNode->FirstChildElement(); child; child = child->NextSiblingElement()) {
		if ( child->ValueStr() == "AgentSet" ) {
			int setClass = 0;
			child->Attribute( "class", &setClass );
			double defaultOr = 0.f;
			child->Attribute( "or", &defaultOr );
			for ( TiXmlElement * grandchild = child->FirstChildElement(); grandchild; grandchild = grandchild->NextSiblingElement() ) {
				if ( grandchild->ValueStr() == "Agent" ) {
					double px = 0.0, py = 0.0, or = 0.0;
					int agtClass = 0;
					grandchild->Attribute( "p_x", &px );
					grandchild->Attribute( "p_y", &py );
					if ( ! grandchild->Attribute( "or", &or ) ) {
						or = defaultOr;
					}
					if ( ! grandchild->Attribute( "class", &agtClass ) ) {
						agtClass = setClass;
					}
					_agents.push_back( new SimAgent( (float)px, (float)py, (float)or, agtClass ) );
				} // Ignore goals				
			}
		} // Ignore obstacles, roadmaps
	}
	_agentsInitd = true;

	return true;
}

////////////////////////////////////////////////////////////////////////////

void PlaybackSimulator::readHeader( bool verbose ) {
	std::string version;
	int agentCount;

	if ( _binaryFile ) {
		char ver[] = "0.0";
		_fileToPlay.read( ver, 4 );
		version = std::string( ver );
	} else {
		_fileToPlay >> version;
	}
	if ( version[0] == '1' ) {
		agentCount = readHeader1( verbose, version[2] );
	} else if ( version[0] == '2' ) {
		agentCount = readHeader2( verbose, version[2] );
	} else {
		std::cerr << "Unknown scb version: " << version << "\n";
		throw SCBVersionError();
	}

	if ( verbose ) {
		std::cout << "Playback file is version " << version << " with " << agentCount << " agents.\n";
		std::cout << "\tAgent size: " << _agentSize << "\n";
	}

	if ( !_agentsInitd ) {
		// If I'm reading the header and my agents haven't been initialized
		createNAgents( agentCount );
		_agentsInitd = true;
	}
}

////////////////////////////////////////////////////////////////////////////

int PlaybackSimulator::readHeader1( bool verbose, char subVersion ) {
	if ( subVersion != '0' ) {
		// only 1.0 is supported for major version 1
		std::cerr << "Unknown scb version: 1." << subVersion << "\n";
		throw SCBVersionError();
	}
	std::string version;
	int agentCount;

	if ( _binaryFile ) {
		_fileToPlay.read( (char*)&agentCount, sizeof(agentCount) );
	} else {
		_fileToPlay >> agentCount;
	}
	_agentSize = 3;	// 2-floats for position, 1 for orientation
	return agentCount;
}

////////////////////////////////////////////////////////////////////////////

int PlaybackSimulator::readHeader2( bool verbose, char subVersion ) {
	std::string version;
	int agentCount;
	if ( subVersion == '0' ) {
		_agentSize = 3;	// 2-floats for position, 1 for orientation
	} else if ( subVersion == '1' ) {
		_agentSize = 4;	// 2-floats for position, 1 for orientation, 1 for state
	} else if ( subVersion == '2' ) {
		_agentSize = 8; // adds pref vel and vel to version 2.1
	} else if ( subVersion == '3' ) {
		// NOTE:
		//	This doesn't support playback of scb version 2.3.  The orientation
		//	is given as a unit vector instead of orientation.  So, it's simpler
		//	to not support it.  It provides no unique benefit to this context.
		//	Simply use version 2.0 if position and orientation are the only required
		//	value.
		std::cerr << "This visualizer does not support scb version 2.3.  Use version 2.0 instead.\n";
		throw SCBVersionError();
	} else if ( subVersion == '4' ) {
		_agentSize = 4;	// 3-floats for position, 1 for orientation
		_posIs3D = true;
	} else {
		std::cerr << "Unknown scb version: 2." << subVersion << "\n";
		throw SCBVersionError();
	}

	if ( _binaryFile ) {
		_fileToPlay.read( (char*)&agentCount, sizeof(agentCount) );
		_fileToPlay.read( (char*)&_simStepLen, sizeof(float) );
	} else {
		_fileToPlay >> agentCount;
		_fileToPlay >> _simStepLen;
	}
	if ( _classIDs == 0x0 ) {
		_classIDs = new int[ agentCount ];
	}
	if ( _binaryFile ) {
		_fileToPlay.read( (char*)&_classIDs[0], sizeof(int) * agentCount );
	} else {
		for ( int i = 0; i < agentCount; ++i ) {
			_fileToPlay >> _classIDs[i];
		}
	}
	

	return agentCount;
}

////////////////////////////////////////////////////////////////////////////

bool PlaybackSimulator::initSimulation( const std::string & playbackFileName ) {
	// Determine if the file indicates binary
	_binaryFile = playbackFileName[ playbackFileName.length() - 1 ] == 'b';
	if ( _binaryFile ) {
		_fileToPlay.open( playbackFileName.c_str(), std::ios::in | std::ios::binary );
	} else {
		_fileToPlay.open( playbackFileName.c_str(), std::ios::in );
	}

	if ( !_fileToPlay ) {
		return false;
	}
	_fileName = playbackFileName;

	try {
		readHeader( true/*verbose*/ );
	} catch ( SCBVersionError  ) {
		return false;
	}
	doStep();

	return true;
}

////////////////////////////////////////////////////////////////////////////

void PlaybackSimulator::clear() {
	if ( _fileToPlay.is_open() ) {
			_fileToPlay.clear();
			_fileToPlay.seekg( 0, std::ios_base::beg );	// rewind the file
			readHeader( false/*verbose*/ );
			doStep();
	}
}

////////////////////////////////////////////////////////////////////////////

bool PlaybackSimulator::doStep() {
	float x, y, o;
	bool err = false;
	const unsigned int BUFF_SIZE = 255;
	char buffer[BUFF_SIZE+1];
	const unsigned int UNUSED = 16;
	float data[ UNUSED ];	// Cache the unused floats in the per-agent data

	const float invStep = 1.f / SimAgent::TIME_STEP;
	if ( _posIs3D ) {
		// I'm only reading the two bytes of position, I need to flush the rest of the bytes
		const unsigned int deltaBytes =  ( _agentSize - 4 ) * sizeof( float );
		assert( (deltaBytes << 2) <= UNUSED && "Insufficient memory allocated for the SCB version" );
		float z;
		for ( size_t i = 0; i < _agents.size(); ++i ) {
			if ( _binaryFile ) {
				if ( ! _fileToPlay.read( (char*)&x, sizeof(float) ) ) err = true;
				if ( ! _fileToPlay.read( (char*)&y, sizeof(float) ) ) err = true;
				if ( ! _fileToPlay.read( (char*)&z, sizeof(float) ) ) err = true;
				if ( ! _fileToPlay.read( (char*)&o, sizeof(float) ) ) err = true;
				if ( deltaBytes ) 
					if ( ! _fileToPlay.read( (char*)&data[0], deltaBytes ) ) 
						err = true;
			} else {
				if ( !( _fileToPlay >> x >> y >> z >> o ) ) {
					err = true;
				}
				_fileToPlay.getline( buffer, BUFF_SIZE );
			}
			if ( err ) {
				std::cerr << "Error reading file starting at agent " << i << ".  Stopping evaluation.\n";
				return false;
			}
			Vector2 vel = ( Vector2( x, z ) - _agents[ i ]->_p ) * invStep;
			_agents[ i ]->setVelocity( vel );
			_agents[ i ]->setPosX( x );
			_agents[ i ]->setPosY( z );
			_agents[ i ]->setHeight( y );
			_agents[ i ]->setOrient( o );
		}
	} else {
		// I'm only reading the two bytes of position, I need to flush the rest of the bytes
		const unsigned int deltaBytes =  ( _agentSize - 3 ) * sizeof( float );
		assert( (deltaBytes << 2) <= UNUSED && "Insufficient memory allocated for the SCB version" );

		for ( size_t i = 0; i < _agents.size(); ++i ) {
			if ( _binaryFile ) {
				if ( ! _fileToPlay.read( (char*)&x, sizeof(float) ) ) err = true;
				if ( ! _fileToPlay.read( (char*)&y, sizeof(float) ) ) err = true;
				if ( ! _fileToPlay.read( (char*)&o, sizeof(float) ) ) err = true;
				if ( deltaBytes ) 
					if ( ! _fileToPlay.read( (char*)&data[0], deltaBytes ) ) 
						err = true;
			} else {
				if ( !( _fileToPlay >> x >> y >> o ) ) {
					err = true;
				}
				_fileToPlay.getline( buffer, BUFF_SIZE );
			}
			if ( err ) {
				std::cerr << "Error reading file starting at agent " << i << ".  Stopping evaluation.\n";
				return false;
			}
			Vector2 vel = ( Vector2( x, y ) - _agents[ i ]->_p ) * invStep;
			_agents[ i ]->setVelocity( vel );
			_agents[ i ]->setPosX( x );
			_agents[ i ]->setPosY( y );
			_agents[ i ]->setOrient( o );
		}
	}

	return !_fileToPlay.eof();

}

////////////////////////////////////////////////////////////////////////////

void PlaybackSimulator::createNAgents( int count ) {
	_agents.clear();
	_agents.resize( count );
	for ( int i = 0; i < count; ++i ) {
		_agents[ i ] = new SimAgent();
		if ( _classIDs != 0x0 ) {
			_agents[i]->setClassID( _classIDs[i] );
		}
	}
}

////////////////////////////////////////////////////////////////////////////
//                IMPLEMENTATION for InterpPlaybackSimulator
////////////////////////////////////////////////////////////////////////////

InterpPlaybackSimulator::InterpPlaybackSimulator(): PlaybackSimulator(), _accumTime(0.f), _frameSize(0), _nextPos(0x0), _moreFrames(true) {
	std::cout << "Interpolation simulator!\n";
}

////////////////////////////////////////////////////////////////////////////

InterpPlaybackSimulator::~InterpPlaybackSimulator() {
	PlaybackSimulator::~PlaybackSimulator();
	if ( _nextPos ) {
		delete [] _nextPos;
	}
}

////////////////////////////////////////////////////////////////////////////

void InterpPlaybackSimulator::createNAgents( int count ) {
	_agents.clear();
	_agents.resize( count );
	for ( int i = 0; i < count; ++i ) {
		_agents[ i ] = new InterpSimAgent();
		if ( _classIDs != 0x0 ) {
			_agents[i]->setClassID( _classIDs[i] );
		}
	}
	_frameSize = _agentSize * _agents.size();
	_nextPos = new float[ _frameSize ];
}

////////////////////////////////////////////////////////////////////////////

bool InterpPlaybackSimulator::initSimulation( const std::string & playbackFileName ) {
	// Determine if the file indicates binary
	_binaryFile = playbackFileName[ playbackFileName.length() - 1 ] == 'b';
	if ( _binaryFile ) {
		_fileToPlay.open( playbackFileName.c_str(), std::ios::in | std::ios::binary );
	} else {
		_fileToPlay.open( playbackFileName.c_str(), std::ios::in );
	}

	if ( !_fileToPlay ) {
		return false;
	}
	_fileName = playbackFileName;

	try {
		readHeader( true/*verbose*/ );
	} catch ( SCBVersionError ) {
		return false;
	}

	// initialize:
	//	1. Update cache
	updateCache();
	//	2. Update agent configuration from cache
	updateAgentFromCache();
	//	3. Update cache
	updateCache();
	//	4. Compute velocities
	updateAgentVelocity();

	return true;
}

////////////////////////////////////////////////////////////////////////////

bool InterpPlaybackSimulator::updateCache() {
	bool err = false;
	const unsigned int BUFF_SIZE = 255;
	char buffer[ BUFF_SIZE + 1 ];

	if ( _binaryFile ) {
		if ( ! _fileToPlay.read( (char*)_nextPos, sizeof( float ) * _frameSize ) ) {
			err = true;
		}
	} else {
		for ( size_t i = 0; i < _agents.size() * _agentSize; i += _agentSize ) {
			if ( !( _fileToPlay >> _nextPos[i] >> _nextPos[i+1] ) ) {
				err = true;
			}
			// Flush the rest of the agent data
			_fileToPlay.getline( buffer, BUFF_SIZE );
		}
	}
	if ( err ) {
		std::cerr << "Error reading file.  Stopping evaluation.\n";
		return false;
	}

	return !_fileToPlay.eof();
}

////////////////////////////////////////////////////////////////////////////

void InterpPlaybackSimulator::updateAgentFromCache() {
	if ( _posIs3D ) {
		for ( size_t i = 0; i < _agents.size(); ++i ) {
			size_t idx = i * _agentSize;
			SimAgent * agt = _agents[ i  ];
			agt->setPosX( _nextPos[ idx ] );
			agt->setHeight( _nextPos[ idx + 1 ] );
			agt->setPosY( _nextPos[ idx + 2 ] );
			agt->setOrient( _nextPos[ idx + 3 ] );
		}
	} else {
		for ( size_t i = 0; i < _agents.size(); ++i ) {
			size_t idx = i * _agentSize;
			SimAgent * agt = _agents[ i  ];
			agt->setPosX( _nextPos[ idx ] );
			agt->setPosY( _nextPos[ idx + 1 ] );
			agt->setOrient( _nextPos[ idx + 2 ] );
		}
	}
}

////////////////////////////////////////////////////////////////////////////

void InterpPlaybackSimulator::updateAgentVelocity() {
	float invStep = 1.0f / _simStepLen;
	
	if ( _posIs3D ) {
		for ( size_t i = 0; i < _agents.size(); ++i ) {
			size_t idx = i * _agentSize;
			InterpSimAgent * agt = dynamic_cast< InterpSimAgent *>( _agents[ i ] );
			Vector2 vel = ( Vector2( _nextPos[ idx ], _nextPos[idx+2] ) - agt->_p ) * invStep;
			agt->setVelocity( vel );
			agt->setVelHeight( ( _nextPos[ idx + 1 ] - agt->_height ) * invStep );
			agt->setAngularVelocity( ( _nextPos[idx+3] - agt->_or ) * invStep );
		}
	} else {
		for ( size_t i = 0; i < _agents.size(); ++i ) {
			size_t idx = i * _agentSize;
			InterpSimAgent * agt = dynamic_cast< InterpSimAgent *>( _agents[ i ] );
			Vector2 vel = ( Vector2( _nextPos[ idx ], _nextPos[idx+1] ) - agt->_p ) * invStep;
			agt->setVelocity( vel );
			agt->setAngularVelocity( ( _nextPos[idx+2] - agt->_or ) * invStep );
		}
	}
}

////////////////////////////////////////////////////////////////////////////

bool InterpPlaybackSimulator::doStep() {
	float elapsed = SimAgent::TIME_STEP;

	//if ( _frameRendered > 100 ) return false;
	_accumTime += elapsed;

	while ( _accumTime >= _simStepLen ) {
		if ( ! _moreFrames ) {
			return false;
		}
		_accumTime -= _simStepLen;
		elapsed = _accumTime;
		updateAgentFromCache();
		_moreFrames = updateCache();
		if ( elapsed < _simStepLen ) {
			updateAgentVelocity();
		}
	}
	
	for ( size_t i = 0; i < _agents.size(); ++i ) {
		InterpSimAgent * agt = dynamic_cast< InterpSimAgent *>( _agents[ i ] );
		agt->_p += agt->_v * elapsed; 
	}

	return true;
}

////////////////////////////////////////////////////////////////////////////

bool InterpPlaybackSimulator::getReachedGoal() const {
	return ! _moreFrames && ( _accumTime >= _simStepLen );
}

////////////////////////////////////////////////////////////////////////////

void InterpPlaybackSimulator::clear() {
	if ( _fileToPlay.is_open() ) {
		_accumTime = 0.f;
		_moreFrames = true;
		_fileToPlay.clear();
		_fileToPlay.seekg( 0, std::ios_base::beg );	// rewind the file
		readHeader();
		
		updateCache();
		updateAgentFromCache();
		updateCache();
		updateAgentVelocity();			
	}
}
