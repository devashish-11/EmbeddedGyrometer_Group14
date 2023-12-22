#ifndef AUDIOOBJ_H
#define AUDIOOBJ_H

#include <stdexcept>
#include "location.h"
#include "velocity.h"
#include "complextype.h"
#include "CircularBuffer.h"
#include "WavObject.h"

using namespace std;

// TODO: move CIRC_BUF_SIZE to Sonic.h
#define BUFFER_CAPACITY 65000 // This will be capacity of circular buffer and also size of memory allocated in wavObject

class AudioObj {
	
private:
    
    friend class Mixer3D;
    friend class World;
    
	Location location;
    Velocity velocity;
    
    bool active;
    float volume;
    bool repeat;
    bool isCompleted; // true if a non-repeated object has completed, false otherwise
    bool backgroundObject; // if true, not mixed binaurally
    bool gpsObject; // TODO: better way with inheritance and polymorphism?
    
    CircularBuffer<Complex> circularBuffer;
    WavObject wavObject;

    void loadCircularBuffer();
    bool fillAudioData(Complex *, unsigned int);

    
public:

	// Creates a new audio object at the world's origin, {0,0,0}.
    
    // TODO: Get rid of hard-coded values in AudioObj constructor
    AudioObj(const std::string wavFileName, bool isBackgroundIn=false) : active(true), isCompleted(false), volume(1), repeat(true), circularBuffer(BUFFER_CAPACITY), wavObject(BUFFER_CAPACITY, wavFileName), backgroundObject(isBackgroundIn) {
        wavObject.loadMoreData(32768, repeat);
        circularBuffer.write(wavObject.complexTempData, 32768);
    }

    AudioObj(const std::string wavFileName, const Location& loc, bool isGpsObjectIn=false) : location(loc), active(true), isCompleted(false), volume(1), repeat(true), circularBuffer(BUFFER_CAPACITY), wavObject(BUFFER_CAPACITY, wavFileName), gpsObject(isGpsObjectIn) {
        wavObject.loadMoreData(32768, repeat);
        circularBuffer.write(wavObject.complexTempData, 32768);
    }
    
    // TODO: Support Gps audio objects with velocity
    // TODO: Condense AudioObj constructors using optional, default parameters.
	// Creates a new audio object at the location specified by the parameter.
    AudioObj(const std::string wavFileName, const Location& loc, const Velocity& vel) : location(loc), velocity(vel), active(true), isCompleted(false), volume(1), repeat(true), circularBuffer(BUFFER_CAPACITY), wavObject(BUFFER_CAPACITY, wavFileName) {
        wavObject.loadMoreData(32768, repeat);
        circularBuffer.write(wavObject.complexTempData, 32768);
    }
    
    ~AudioObj () { }
	
	// Returns the array of the object's location.
	Location getLocation() const;
    
    // Changes the object's location to that which is specifies in the parameter.
	void setLocation (const Location& loc);
	void setLocation (float x, float y, float z);
    
    Velocity getVelocity() const;

    void setVelocity (const Velocity& vel);
    void setVelocity (float dx, float dy, float dz);
    
    // Returns the volume of the audio object.
	// This will be a value from 0 to 1.
	float getVolume() const;
    
    // Sets the volume of the audio object. This will only accept values from 0 to 1.
	void setVolume(float vol);

	// Sets the volume of the audio object to a random value in the range
	// [0.01, 1].
	void setRandomVolume();

	// Returns whether or not the object is active
	bool isActive() const;

	// Changes whether or not the object is active
	void setActive(bool active);
    
    bool isGpsObject() const;
    
    bool isBackgroundObject() const;
    
    void setRepeat(bool rep);
    
    // Plays audio object exactly once. Reloads input buffer if necessary. Will restart if currently playing.
    void playOnceFromBeginning();
    
    // Resets to the beginning and activates
    void restart();
    
    // For pre-loading, but not yet starting, from the beginning
    void loadFromBeginning();
};

#endif
