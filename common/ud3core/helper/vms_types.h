/*
    Copyright (C) 2020,2021 TMax-Electronics.de
   
    This file is part of the MidiStick Firmware.

    the MidiStick Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    the MidiStick Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with the MidiStick Firmware.  If not, see <https://www.gnu.org/licenses/>.
*/
#if PIC32
    #include <xc.h>
#endif
#include <stdint.h>

#if !defined(vms_types_H)
#define vms_types_H

#define VMS_MAX_BRANCHES 4

#define NO_BLK (VMS_BLOCK*)0xFFFFFFFF
#define HARD_BLK(x) (VMS_BLOCK*)x

typedef struct _SynthVoice_ SynthVoice;
typedef struct _ChannelInfo_ ChannelInfo;
typedef struct _SINE_DATA_ SINE_DATA;
typedef struct _VMS_listDataObject_ VMS_listDataObject;
typedef struct _VMS_BLOCK_ VMS_BLOCK;
typedef struct _MapData_ MAPTABLE_DATA;
typedef struct _MapEntry_ MAPTABLE_ENTRY;
typedef struct _MapHeader_ MAPTABLE_HEADER;

typedef enum {TONE_NORMAL, TONE_NOISE, TONE_SINGE_SHOT} ToneType;                       // TODO: None of these are currently used
typedef enum {MOD_AM_NOISE, MOD_FM_NOISE, MOD_AM_SWEEP, MOD_FM_SWEEP} ModulationType;   // TODO: None of these are currently used

// Modulation operations for the VMS_BLOCK's
typedef enum {VMS_EXP, VMS_EXP_INV, VMS_LIN, VMS_SIN, VMS_JUMP} VMS_MODTYPE;            

// Specifies the value to modify in the VMS_BLOCK
typedef enum {maxOnTime, minOnTime, onTime, otCurrent, otTarget, otFactor, frequency, freqCurrent, freqTarget, freqFactor, noise, pTime, circ1, circ2, circ3, circ4, CC_102, CC_103, CC_104, CC_105, CC_106, CC_107, CC_108, CC_109, CC_110, CC_111, CC_112, CC_113, CC_114, CC_115, CC_116, CC_117, CC_118, CC_119, HyperVoice_Count, HyperVoice_Phase, KNOWNVAL_MAX} KNOWN_VALUE;

// When to apply the VMS_BLOCK operation
typedef enum {RISING, FALLING, ANY, NONE} DIRECTION;

// TODO: Only used in VMS_calculateValue()
typedef enum {INVERTED = 0, NORMAL = 1} NOTEOFF_BEHAVIOR;

typedef struct _DLLObject_ DLLObject;

// A doubly linked list node
struct _DLLObject_{
    DLLObject * next;
    DLLObject * prev;
    uint32_t uid;
    void * data;
};

// Defines how to play a range of notes (condensed down to a single frequency?)
struct _MapData_{
    int16_t noteFreq;
    uint8_t targetOT;
    uint8_t flags;
    VMS_BLOCK * VMS_Startblock;     //this is the VMS_Block ID when in flash, and will be changed to be the pointer to the block when loaded into ram
}__attribute__((packed));

// A variable number of these immediately follow the _MapHeader_ struct.  These define
// the way sounds in a given range of notes are played.   Notes in the range startNote
// to endNote are played with the values defined by data.
struct _MapEntry_{
    uint8_t startNote;      // Lower limit of range of notes
    uint8_t endNote;        // Upper limit of range of notes
    MAPTABLE_DATA data;     // Defines how to play the notes
}__attribute__((packed));

// There is one _MapHeader_ for each MIDI program.  A "program" is a set of sounds, effects, 
// etc and can be thought of as an instrument. The program data is stored in flash for reference 
// while the MIDI is playing
struct _MapHeader_{
    uint8_t listEntries;            // The number of MAPTABLE_ENTRY's that immediately follow this struct
    uint8_t programNumber;          // The MIDI program (instrument) number.
    char name[18];
} __attribute__((packed));

//struct to contain all variables and parameters for the voices
struct _SynthVoice_{
    uint32_t    freqTarget;
    uint32_t    freqCurrent;
    int32_t     freqFactor;
    
    uint32_t    otTarget;
    uint32_t    otCurrent;
    int32_t     otFactor;
    
    uint32_t    noiseTarget;
    uint32_t    noiseCurrent;
    uint32_t    noiseRaw;
    int32_t     noiseFactor;
    
    uint8_t     currNote;               // The current note being played (the key number)
    uint8_t     hyperVoiceCount;
    uint8_t     hyperVoicePhase;
    int32_t     hyperVoicePhaseFrac;
    uint16_t    hyperVoice_timings[2];
    uint32_t    on;                     // 0 indicates note is off
    uint32_t    noteAge;
    uint8_t     currNoteOrigin;         // The channel for the note
    int32_t     portamentoParam;
    
    int32_t     circ1;
    int32_t     circ2;
    int32_t     circ3;
    int32_t     circ4;
    
    uint8_t     id;
    MAPTABLE_DATA * map;
};

struct _ChannelInfo_{
    uint32_t        bendFactor;   
    
    unsigned        sustainPedal;
    unsigned        damperPedal;
    
    uint16_t        bendMSB;   
    uint16_t        bendLSB;
    float           bendRange;
    
    uint16_t        currOT;
    int32_t         portamentoTime;     // 0-127 in .7 fixed point?
    uint32_t        lastFrequency;
    uint8_t         parameters[18];
    
    uint16_t        currVolume;         // Channel volume (0-127)
    uint16_t        currStereoVolume;
    uint8_t         currStereoPosition; // 0-127
    uint16_t        currVolumeModifier;
    
    uint8_t         currProgramm;       // The current program (set of sounds, effects, etc)
    MAPTABLE_HEADER * currentMap;       // A pointer to the struct that contains all the data for currProgramm
};

// TODO: a pointer to one of these is stored in _VMS_listDataObject_.data
struct _SINE_DATA_{
    int32_t currCount;
};

// A pointer to this struct is stored in the doubly linked list object's (_DLLobject) .data member.
struct _VMS_listDataObject_{
    SynthVoice * targetVoice;
    VMS_BLOCK * block;
    void * data;                        // May point to a SINE_DATA
    
    DIRECTION thresholdDirection;
    uint32_t nextRunTime;
    uint32_t period;
};

// This applies a linear interpolation of the value from rangeStart to rangeEnd.  This
// is used in VMS_BLOCK for the 4 parameters.
typedef struct{
    KNOWN_VALUE sourceId    : 8;        // First 8 bits equal the value to modify
    int32_t     rangeStart  : 12;       // Next 12 bits = start
    int32_t     rangeEnd    : 12;       // High 12 bits = end
} RangeParameters;

// From Thorben:
// A VMS_BLOCK contains instructions describing how the ud3 should modulate the sound. 
// The blocks are part of a singly linked list and thus have a pointer to the next one 
// to be loaded once the value to be modulated has reached the target factor. Targets 
// can be named with the enum KnownValues, each element in which names a parameter (for 
// example note frequency). Everyone of those blocks is effectively an instruction that 
// describes a math operation to do to that target. The VMS_EXP block for example multiplies 
// the current value with a factor and then writes the result of that operation to the targetValue.
struct _VMS_BLOCK_{
    uint32_t uid;
    VMS_BLOCK * nextBlocks[VMS_MAX_BRANCHES];   // These allow other values to be modified when this value is changed.
    VMS_BLOCK * offBlock;                       // Activated when the Note Off command is processed
    NOTEOFF_BEHAVIOR behavior;
    VMS_MODTYPE type;                           // One of the modulation types listed above (defines the operation to perform)
    KNOWN_VALUE target;                         // The value to modify
    DIRECTION thresholdDirection;               // One of the DIRECTION enums to specify the direction of the curve for the operation
    int32_t targetFactor;                       // Either a parameter or a RangeParameters struct (if flags & VMS_FLAG_ISVARIABLE_TARGETFACTOR)
    int32_t param1;                             // Either a parameter or a RangeParameters struct (if flags & VMS_FLAG_ISVARIABLE_PARAM1)
    int32_t param2;                             // Either a parameter or a RangeParameters struct (if flags & VMS_FLAG_ISVARIABLE_PARAM2)
    int32_t param3;                             // Either a parameter or a RangeParameters struct (if flags & VMS_FLAG_ISVARIABLE_PARAM3)
    uint32_t period;
    uint32_t flags;
};
#endif