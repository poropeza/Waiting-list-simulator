#ifndef __SIMULATOR_H__
#define __SIMULATOR_H__

#include <fstream>
#include <vector>
#include <string>
#include "math/vector.h"
#include "exception"

// Base simulator class
//		- presents the minimum interface which the Horde3D visualizer requires

class SimAgent {
public:
	SimAgent():_p(0.f,0.f), _v(0.f,0.f), _height(0.f), _or(0.f), _class(0) {}
	SimAgent( float x, float y ):_p(x,y), _v(0.f,0.f), _or(0.f), _class(0) {}
	SimAgent( float x, float y, float orient ):_p(x,y), _v(0.f,0.f), _or(orient), _class(0) {}
	SimAgent( float x, float y, float orient, int c ):_p(x,y), _v(0.f,0.f), _or(orient), _class(c) {}

	virtual ~SimAgent() {}

	inline void setPosX( float x ) { _p.x = x; }
	inline float getPosX() const { return _p.x; }
	inline void setPosY( float y ) { _p.y = y; }
	inline float getPosY() const { return _p.y; }
	inline void setHeight( float h ) { _height = h; }
	inline float getHeight() const { return _height; }
	inline void setOrient( float orient ) { _or = orient; }
	inline float getOrient() const { return _or; }
	inline void setClassID( int id ) { _class = id; }
	inline int getClassID() const { return _class; }
	inline void setVelocity( const Vector2 & vel ) { _v = vel; }
	inline Vector2 getVelocity() { return _v; }
	inline void setVelX( float x ) { _v.x = x; }
	inline float getVelX() { return _v.x; }
	inline void setVelY( float y ) { _v.y = y; }
	inline float getVelY() { return _v.y; }
	inline float getSpeed() const { return _v.Length(); }

	float getTimeStep() const { return TIME_STEP; }

	// Position of agent (in R2)
	Vector2 _p;

	// Velocity of agent (in R2)
	Vector2 _v;

	// Height of agent
	float _height;

	// Orientation of agent (in radians)
	float   _or;

	// Visualization classification
	int     _class;

	static float TIME_STEP;
};

// A simulation agent that stores its velocity and can maintain the speed between simulation
// steps.
class InterpSimAgent : public SimAgent {
public:
	InterpSimAgent(): SimAgent(), _dHeight(0.f), _omega(0.f) {}
	InterpSimAgent( float x, float y ): SimAgent(x,y), _omega(0.f) {}
	InterpSimAgent( float x, float y, float orient ): SimAgent(x,y,orient), _omega(0.f) {}
	InterpSimAgent( float x, float y, float orient, int c ): SimAgent(x,y,orient,c), _omega(0.f) {}

	inline void setVelHeight( float h ) { _dHeight = h; }
	inline float getVelHeight() const { return _dHeight; }
	inline void setAngularVelocity( float omega ) { _omega = omega; }
	inline float getAngularVelocity() { return _omega; }

	inline void updatePosition( float dt ) { _p += (_v * dt ); _or += (_omega * dt); _height += ( _dHeight * dt ); }

	// Velocity of agent in altitude
	float _dHeight;

	// The angular velocity of the agent -- the change in orientation
	float	_omega;
};

typedef std::vector< SimAgent * > AgentVec;
typedef AgentVec::iterator AgentVecItr;
typedef AgentVec::const_iterator AgentVecCItr;

class SimulatorError : public std::exception {
};


class Simulator {
public:
	Simulator(): _posIs3D(false) {}
	virtual ~Simulator() {}

	// Informs the simulator that the system is starting to run, or pausing its run
	virtual void start() {}
	virtual void pause() {}

	// Initializes the simulator from the scene-definition XML
	//
	virtual bool loadFromXML( const std::string & fileName ) = 0;

	// Initialize the simulation
	//
	virtual bool initSimulation() { return true; }

	// reset the simulation to initial conditions
	//
	virtual void clear() = 0;
	
	// reports if the simulation is complete (i.e. all agents have reached goal)
	//
	virtual bool getReachedGoal() const = 0;

	// Sets the size of the simulator's timestep
	//
	void setTimeStep( float stepSize ) { SimAgent::TIME_STEP = stepSize; }

	// Causes the simulator to step forward in simulation.
	//	returns a boolean reporting if another step is possible
	//
	virtual bool doStep() = 0;

	// Returns the number of agents in the simulation
	//
	virtual int getNumAgents() const = 0;

	// Returns a pointer to the ith agent
	//
	virtual SimAgent* getAgent( int index ) = 0;

protected:
	// Determines if the simulator data has 3D position data
	//
	bool	_posIs3D;
};

class SCBVersionError : public SimulatorError {
};

class PlaybackSimulator: public Simulator {
public:
	PlaybackSimulator(): Simulator(), _agentsInitd( false ), _binaryFile( false ), _fileName(""), _classIDs(0x0), _agentSize(3), _simStepLen(0.1f) {}
	virtual ~PlaybackSimulator();

	// Initializes the simulator from the scene-definition XML
	//
	virtual bool loadFromXML( const std::string & fileName );

	// Initializes with the name of the file containing the playback data
	//
	bool initSimulation( const std::string & playbackFileName );

	// reset the simulation to initial conditions
	//
	virtual void clear();
	
	// reports if the simulation is complete (i.e. all agents have reached goal)
	//
	virtual bool getReachedGoal() const { return _fileToPlay.eof(); }

	// Causes the simulator to step forward in simulation
	//
	virtual bool doStep();

	// Returns the number of agents in the simulation
	//
	virtual int getNumAgents() const { return (int)_agents.size(); }

	// Returns a pointer to the ith agent
	//
	virtual SimAgent* getAgent( int index ) { return _agents[ index ]; }

protected:
	// State which reports if the agents have been initialized
	//
	bool _agentsInitd;

	// remembers if the playback file is binary (ascii if the value is false)
	//
	bool _binaryFile;

	// file name
	//
	std::string _fileName;

	// An array of classes for the agents -- one class ID per agent
	//	If the pointer is null, then they are all class 0
	int	*	_classIDs;
	
	// The size of a single agent's data in FLOATS - it can vary depending on the
	//	file version
	unsigned int	_agentSize;

	// The time step inherent in the simulation data
	float	_simStepLen;

	// Vector of the agents in the simulation
	//
	AgentVec _agents;

	// file stream for the playback file
	//
	std::ifstream _fileToPlay;

	// Reads the header of the file
	//
	void readHeader( bool verbose = false );
	// read the header for version 0.0 - returns the number of agents
	int readHeader1( bool verbose, char subVersion );
	// read the header for version 1.0 - returns the number of agents
	int readHeader2( bool verbose, char subVersion );

	// Creates n agents
	//
	virtual void createNAgents( int count );
};


class InterpPlaybackSimulator: public PlaybackSimulator {
public:
	InterpPlaybackSimulator();
	virtual ~InterpPlaybackSimulator();

	// Initializes with the name of the file containing the playback data
	//
	bool initSimulation( const std::string & playbackFileName );

	// reset the simulation to initial conditions
	//
	virtual void clear();
	
	// reports if the simulation is complete (i.e. all agents have reached goal)
	//
	virtual bool getReachedGoal() const;// { return _fileToPlay.eof(); }

	// Causes the simulator to step forward in simulation
	//		Returns true if there are more steps possible
	//
	virtual bool doStep();

protected:
	// Amount of time accumulated since last simulation step read
	//
	float _accumTime;

	// The size of a frame of agent data in bytes - it can vary depending on the
	//	file version
	unsigned int	_frameSize;

	// array with 3N elements: x,y positions and orientation of N agents
	//	used to compute velocities.
	float * _nextPos;

	// Remembers if there are more frames to read from the simulation
	//
	bool _moreFrames;

	// update the cache with data from the file
	//	returns true if more data is available, false otherwise
	//
	bool updateCache();

	// updates the agent configuration from the cache
	//
	void updateAgentFromCache();

	// Computes the agent velocities based on current agent position and cached position
	//
	void updateAgentVelocity();

	// Creates n agents
	//
	void createNAgents( int count );
};


#endif	//__SIMULATOR_H__
