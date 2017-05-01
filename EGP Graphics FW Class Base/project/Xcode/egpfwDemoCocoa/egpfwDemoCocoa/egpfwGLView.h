//
//  egpfwGLView.h
//  egpfwDemoCocoa
//
//  Created by Dan Buckstein on 2017-01-28.
//  Copyright © 2017 Dan Buckstein. All rights reserved.
//

#ifndef __EGPFW_GLVIEW_H
#define __EGPFW_GLVIEW_H


#import <Cocoa/Cocoa.h>

#import <CoreVideo/CoreVideo.h>


@interface egpfwGLView : NSOpenGLView
{
    CVDisplayLinkRef displayLink;
}

- (void)drawRect:(NSRect)bounds;

- (CVReturn)getFrameForTime:(const CVTimeStamp*)outputTime;

@end


#endif  // __EGPFW_GLVIEW_H
