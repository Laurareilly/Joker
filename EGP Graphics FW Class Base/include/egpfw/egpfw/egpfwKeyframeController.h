/*
	EGP Graphics Framework
	(c) 2017 Dan Buckstein
	General keyframe management utilities by Dan Buckstein

	Modified by: ______________________________________________________________
*/

#ifndef __EGPFW_KEYFRAMECONTROLLER_H
#define __EGPFW_KEYFRAMECONTROLLER_H


#ifdef __cplusplus
extern "C"
{
#endif	// __cplusplus


//-----------------------------------------------------------------------------
// enumerators
	
	enum egpSequenceTransition
	{
		ANIM_STOP,	// animation should stop when it hits the end
		ANIM_LOOP,	// animation should loop when it hits the end
		ANIM_GOTO,	// animation should go to another when it hits the end
	};

#ifndef __cplusplus
	typedef enum egpSequenceTransition egpSequenceTransition;
#endif	// __cplusplus


//-----------------------------------------------------------------------------
// data structures

	// forward declaration
	struct egpKeyframeSequenceDescriptor;

#ifndef __cplusplus
	typedef struct egpKeyframeSequence				egpKeyframeSequence;
	typedef struct egpKeyframeSequenceDescriptor	egpKeyframeSequenceDescriptor;
	typedef struct egpKeyframeController			egpKeyframeController;
#endif	// __cplusplus

	// sequence structure to manage frame ranges
	struct egpKeyframeSequence
	{
		unsigned int first, last, count;
		float durationSeconds;
		float framesPerSecond;
		float secondsPerFrame;
		const egpKeyframeSequenceDescriptor *descriptor;
		egpKeyframeSequence *startGoToSequence, *endGoToSequence;
		egpSequenceTransition startTransition, endTransition;
		char name[16];
	};

	// sequence structure to manage frame ranges
	struct egpKeyframeSequenceDescriptor
	{
		egpKeyframeSequence *sequences, *startSeq;
		unsigned int numSequences;
	};

	// controller structure
	struct egpKeyframeController
	{
		const egpKeyframeSequenceDescriptor *sequences;
		const egpKeyframeSequence *currentSeq, *nextSeq, *prevSeq;
		unsigned int fPrev, f0, f1, fNext;
		int play;			// play flag
		float frameTime;	// current time in frame
		float frameParam;	// current frame interpolation parameter
	};


//-----------------------------------------------------------------------------
// functions

	// load sequence data
	// returns object with data if valid
	// 'filePath' cannot be null or an empty string
	egpKeyframeSequenceDescriptor egpfwLoadSequenceData(const char *filePath);

	// update controller
	// pass in a time change and let it take care of the rest
	// returns 1 if successful, 0 if failed
	// 'ctrl' param cannot be null and must be initialized
	int egpfwUpdateKeyframeController(egpKeyframeController *ctrl, const float dt);

	// change sequence
	// returns 1 if successful, 0 if failed
	// 'ctrl' and 'seq' params cannot be null and must be initialized
	int egpfwKeyframeControllerSetSequence(egpKeyframeController *ctrl, const egpKeyframeSequence *seq);

	// get a sequence by name from a set of sequences
	// returns valid index or pointer if successful, 
	//	-1 (index) or null (pointer) if failed
	// 'seq' param cannot be null
	// 'name' param cannot be null or an empty string
	int egpfwGetSequenceIndexByName(const egpKeyframeSequenceDescriptor *seq, const char *name);
	const egpKeyframeSequence *egpfwGetSequenceByName(const egpKeyframeSequenceDescriptor *seq, const char *name);

	// release sequence data
	// returns 1 if successful, 0 if failed
	// 'seq' param cannot be null
	int egpfwReleaseSequenceData(egpKeyframeSequenceDescriptor *seq);


//-----------------------------------------------------------------------------


#ifdef __cplusplus
}
#endif	// __cplusplus


#endif	// __EGPFW_KEYFRAMECONTROLLER_H