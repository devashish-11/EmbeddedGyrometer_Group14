#ifndef WORLD_H
#define WORLD_H

#include <stdexcept>
#include <vector>
#include "pthread.h"
#include <unistd.h>

#include "location.h"
#include "velocity.h"
#include "AudioObj.h"
#include "Player.h"

// TODO: Move AUDIO_OBJECT_LOAD_INTERVAL to Sonic.h
#define AUDIO_OBJECT_LOAD_INTERVAL 10000 // interval at which input is loaded into audio object buffers

using namespace std;

// TODO: finish documenting World class
class World {

	Player player;
    vector<AudioObj *> objList;
	float threshold;
    pthread_t writeThread;
    bool isWriteThreadCreated;
    
    static void *writeAudioObjects (void *);

  public:
	static const int MAX_OBJ = 20;
    
    //Constructors
    
	/**
     Default constructor creates a player at the world's origin, {0,0,0}.
	*/
    World() : threshold(0.05), isWriteThreadCreated(false) {}

	/**
     Creates a player at the
	 location specified by the first parameter,
	 and sets the player's bearing specified by
	 the second parameter.
    */
	World(const Location& loc, const Velocity& vel, float bear) : player(Player(loc,vel, bear)), threshold(0.05), isWriteThreadCreated(false) {}

    ~World();

    //Getters
    float getPlayerBearing();

	/**
     Returns a reference to the player.
     */
	Player& getPlayer();

    /**
     Returns a reference to the audio object at the
	 specified index.
     */
	AudioObj* getAudioObj(size_t index) const;
    
	/**
     Returns the array of the player's location.
     */
    Location getPlayerLocation() const;
    void getPlayerLocation(float &xOut, float &yOut, float &zOut);
    void getPlayerGpsLocation(float &latitudeOut, float &longitudeOut, float &altitudeOut);
    /**
     Returns the number of audio objects in the world.
     */
    int  getNumObj();

    // Setters
    void setPlayerLocation(float x, float y, float z);
    void setPlayerGpsLocation(float latitude, float longitude, float altitude);
    void setPlayerBearing(float bearing);
    
	/**
     Adds an audio object to the world. Returns the
	 index of the created object. Sets the location
	 of the created object at the world's origin, {0,0,0}.
     */
    AudioObj* addAudioObj(const std::string wavFileName, bool isBackgroundObject=false);

    
    AudioObj* addAudioObj(const std::string wavFileName, const Location& loc, bool isGpsObject=false);
    
	/**
     Adds an audio object to the world. Returns the
	 index of the created object. Sets the location
	 of the created object at the location specified
	 by the parameter.
     */
    AudioObj* addAudioObj(const std::string wavFileName, const Location& loc, const Velocity& vel);
    
    AudioObj* addBackgroundAudioObj(const std::string wavFileName);
    
    void createWriteThread();


};

#endif
