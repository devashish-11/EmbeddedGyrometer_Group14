//
//  WavObject.h
//  Demo
//
//  Created by Philadelphia Game Lab on 7/7/14.
//  Copyright (c) 2014 Philadelphia Game Lab. All rights reserved.
//

#ifndef WAVOBJECT_H
#define WAVOBJECT_H

#include <string>
#include "complextype.h"
#include <cmath>
#include <math.h>
#include <iostream>

//using std::complex;

class WavObject {
    
    FILE* soundFile;
    struct
    {
        long n;
        int sampleRate;
        int bitDepth;
        int nChannels;
    }wavFileData;
    long startOfWavData;
    long endOfWavData;
    
public:
    
    //Struct that holds RIFF data of Wave File.
    //RIFF data is meta data info that holds ID, size and format of WAVE file.
    struct RIFF_Header {
        char chunkID[4];
        unsigned int chunkSize;
        char format[4];
    };
    
    //Struct that holds format subchunk data for WAVE file.
    struct WAVE_Format {
        unsigned int subChunkSize;
        short audioFormat;
        short nChannels;
        unsigned int sampleRate;
        unsigned int byteRate;
        short blockAlign;
        short bitsPerSample;
    };
    
    //Struct that holds data of WAVE file.
    struct WAVE_Data {
        long subChunk2Size;
    };
    
    //Struct to hold subchunkID
    struct CHUNK_ID {
        char chunkID[4];
    };
    
    
    WAVE_Format wave_format;
    RIFF_Header riff_header;
    WAVE_Data wave_data;
    CHUNK_ID chunk_id;
    
    short *shortTempData;
    Complex *complexTempData;
    
    
    WavObject (unsigned int size,const std::string wavFileName) : shortTempData(new short[size]), complexTempData(new Complex[size]) {
        extractWavHeader(wavFileName);
    }
    
    Complex *loadCmpWavData(const std::string fname, long *size, int *smpFreq, int *bitDepth, int *channels);
    void extractWavHeader (const std::string fname);
    
    // returns false if end of file reached and no repeat, true otherwise
    bool loadMoreData(unsigned int size, bool repeat);
    
    void seekToBeginning();
    
    ~WavObject () {
        delete[] shortTempData;
        delete[] complexTempData;
        if (soundFile)
            fclose(soundFile);
    }
    
};

#endif