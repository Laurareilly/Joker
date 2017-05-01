// By Dan Buckstein
// Modified by: _______________________________________________________________
#include "egpfw/egpfw/egpfwKeyframeController.h"


#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


// ****
// load sequence data
egpKeyframeSequenceDescriptor egpfwLoadSequenceData(const char *filePath)
{
	egpKeyframeSequenceDescriptor seq = { 0 };
	//...
	return seq;
}


// ****
// update controller
int egpfwUpdateKeyframeController(egpKeyframeController *ctrl, const float dt)
{
	//...
	return 0;
}


// ****
// change sequence
int egpfwKeyframeControllerSetSequence(egpKeyframeController *ctrl, const egpKeyframeSequence *seq)
{
	//...
	return 0;
}


// ****
// get a sequence by name
int egpfwGetSequenceIndexByName(const egpKeyframeSequenceDescriptor *seq, const char *name)
{
	//...
	return -1;
}

// ****
const egpKeyframeSequence *egpfwGetSequenceByName(const egpKeyframeSequenceDescriptor *seq, const char *name)
{
	//...
	return 0;
}


// ****
// release sequence data
int egpfwReleaseSequenceData(egpKeyframeSequenceDescriptor *seq)
{
	//...
	return 0;
}
