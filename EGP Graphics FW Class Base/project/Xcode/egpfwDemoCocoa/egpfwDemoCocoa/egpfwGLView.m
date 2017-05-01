//
//  egpfwGLView.m
//  egpfwDemoCocoa
//
//  Created by Dan Buckstein on 2017-01-28.
//  Copyright © 2017 Dan Buckstein. All rights reserved.
//
/*
 Resources:
 https://developer.apple.com/library/content/documentation/Cocoa/Conceptual/EventOverview/HandlingKeyEvents/HandlingKeyEvents.html
 https://developer.apple.com/library/content/documentation/Cocoa/Conceptual/EventOverview/HandlingMouseEvents/HandlingMouseEvents.html
 https://developer.apple.com/library/content/documentation/GraphicsImaging/Conceptual/OpenGL-MacProgGuide/opengl_drawing/opengl_drawing.html
 https://developer.apple.com/library/content/qa/qa1385/_index.html
 https://www.opengl.org/discussion_boards/showthread.php/176673-OpenGL-3-0-context-setup-in-XCode4-using-Lion
 http://stackoverflow.com/questions/7610117/layer-backed-openglview-redraws-only-if-window-is-resized
 */


#include "../source/egpfwDemo/egpfwDemo.h"

#include "egpfw/egpfw/utils/egpfwInputUtils.h"


#import "egpfwGLView.h"


static NSWindow *win_handle;
static const unsigned char win_close = 27;  // escape key
static int context_modernized = 0;
static int win_resizing = 0;

static NSTrackingArea *trackingArea;
static int mouseInView = 0;


@implementation egpfwGLView


// equivalent of onDisplay
static CVReturn displayLinkCallback(CVDisplayLinkRef displayLink, const CVTimeStamp* now, const CVTimeStamp* outputTime, CVOptionFlags flagsIn, CVOptionFlags* flagsOut, void* displayLinkContext)
{
    CVReturn result = [(egpfwGLView*)displayLinkContext getFrameForTime:outputTime];
    return result;
}

// response to mouse click
static void mouseButton(int btn, int state, NSPoint point)
{
    if (mouseInView && !win_resizing)
        egpMouseButtonCallback(btn, state, (unsigned int)(point.x), windowHeight() - (unsigned int)(point.y));
}

// response to close window key
static void winClosePressed()
{
    // cancel all flags
    context_modernized = 0;
    win_resizing = 0;
    mouseInView = 0;
    
    // close window
    [win_handle close];
    win_handle = 0;
}


// equivalent of onIdle... called exactly one time for some sadistic reason...
- (void)drawRect:(NSRect)bounds
{
    // DO NOT USE
}


// draw callback
- (CVReturn)getFrameForTime:(const CVTimeStamp*)outputTime
{
    [[self openGLContext] makeCurrentContext];
    
    if (!win_resizing && idle())
    {
        [[self openGLContext] flushBuffer];
    }
    
    return kCVReturnSuccess;
}

- (void)dealloc
{
    // destroy game objects
    onCloseWindow();
    
    // Release the display link
    CVDisplayLinkRelease(displayLink);
    
    // tracking
    [trackingArea release];
    
    // done
    [super dealloc];
}


// this function is called when the view sets its context
- (void)prepareOpenGL
{
    // check GL version
    printf("\n Current GL context version (modern=%d): \n", context_modernized);
    printGLVersion();
    
    // prepare current context if it is modernized
    if (context_modernized)
    {
        // proceed with regular initialization
        [super prepareOpenGL];
    
        // display update interval
        int swapInterval[1] = { 0 };
        [[self openGLContext] setValues:swapInterval forParameter:NSOpenGLCPSwapInterval];
    
        // Create a display link capable of being used with all active displays
        CVDisplayLinkCreateWithActiveCGDisplays(&displayLink);
    
        // Set the renderer output callback function
        CVDisplayLinkSetOutputCallback(displayLink, &displayLinkCallback, self);
    
        // Set the display link for the current renderer
        CGLContextObj cglContext = [[self openGLContext] CGLContextObj];
        CGLPixelFormatObj cglPixelFormat = [[self pixelFormat] CGLPixelFormatObj];
        CVDisplayLinkSetCurrentCGDisplayFromOpenGLContext(displayLink, cglContext, cglPixelFormat);
    
        // Activate the display link
        CVDisplayLinkStart(displayLink);
    
        // respond to input events
        [[self window] setNextResponder:self];
        win_handle = [self window];
        
        NSTrackingAreaOptions options = (NSTrackingActiveInActiveApp | NSTrackingMouseMoved | NSTrackingMouseEnteredAndExited);
        trackingArea = [[NSTrackingArea alloc] initWithRect:[self bounds] options:options owner:self userInfo:nil];
        [self addTrackingArea:trackingArea];
    
        // initialize graphics states
        if (initGL())
        {
            // initialize image library
            if (initIL())
            {
                // initialize game objects
                if (initGame())
                {
                    displayControls();
                    // force first update and render
                    updateGameState(0.0f);
                    renderGameState();
                    // timers
                    resetTimers();
                    // set close key
                    egpSetCloseKey(win_close, winClosePressed);
                }
            }
        }
    }
    // force modern context
    else
    {
        NSOpenGLPixelFormatAttribute newAttribs[] =
        {
            NSOpenGLPFAColorSize, 64,
            NSOpenGLPFADepthSize, 32,
            NSOpenGLPFAOpenGLProfile, NSOpenGLProfileVersion4_1Core,
            NSOpenGLPFAAccelerated,
            NSOpenGLPFADoubleBuffer,
            NSOpenGLPFABackingStore,
            0
        };
        
        // get old info and create new ones
        NSOpenGLPixelFormat *newFormat = [[NSOpenGLPixelFormat alloc] initWithAttributes:newAttribs];
        NSOpenGLContext *newContext = [[NSOpenGLContext alloc] initWithFormat:newFormat shareContext:nil];
        
        // done, raise context flag
        context_modernized = 1;
        
        // release old format and context
        // set new format and context
        [[self openGLContext] clearDrawable];
        [newContext makeCurrentContext];
        [super setPixelFormat:newFormat];
        [super setOpenGLContext:newContext];
    }
}

- (void)update;		// moved or resized
{
    [super update];
}

- (void)reshape;	// scrolled, moved or resized
{
    [super reshape];
    
    // trigger callbacks if not currently resizing
    if (context_modernized && !win_resizing)
    {
        NSRect bounds = [self bounds];
        [[self openGLContext] makeCurrentContext];
        onPositionWindow((unsigned int)(bounds.origin.x), (unsigned int)(bounds.origin.y));
        onResizeWindow((unsigned int)(bounds.size.width), (unsigned int)(bounds.size.height));
    }
}

- (void)viewWillStartLiveResize
{
    // window is about to be resized
    [super viewWillStartLiveResize];
    [win_handle setIgnoresMouseEvents:YES];
    win_resizing = 1;
}

- (void)viewDidEndLiveResize
{
    // window is finished resizing
    [super viewDidEndLiveResize];
    [win_handle setIgnoresMouseEvents:NO];
    win_resizing = 0;
}


// input
- (void)keyDown:(NSEvent *)event
{
    const unsigned char key = [[event characters] characterAtIndex:0];
    egpKeyboardPressCallback(key, 0, 0);
}

- (void)keyUp:(NSEvent *)event
{
    const unsigned char key = [[event characters] characterAtIndex:0];
    egpKeyboardReleaseCallback(key, 0, 0);
}

- (void)mouseMoved:(NSEvent *)event
{
    if (mouseInView)
    {
        NSPoint point = [event locationInWindow];
        egpMouseMotionCallback((unsigned int)(point.x), windowHeight() - (unsigned int)(point.y));
    }
}

- (void)mouseDown:(NSEvent *)event
{
    mouseButton(0, 0, [event locationInWindow]);
}

- (void)mouseUp:(NSEvent *)event
{
    mouseButton(0, 1, [event locationInWindow]);
}

- (void)rightMouseDown:(NSEvent *)event
{
    mouseButton(2, 0, [event locationInWindow]);
}

- (void)rightMouseUp:(NSEvent *)event
{
    mouseButton(2, 1, [event locationInWindow]);
}

- (void)otherMouseDown:(NSEvent *)event
{
    mouseButton(1, 0, [event locationInWindow]);
}

- (void)otherMouseUp:(NSEvent *)event
{
    mouseButton(1, 1, [event locationInWindow]);
}

- (void)mouseDragged:(NSEvent *)event
{
    [self mouseMoved:event];
}

- (void)rightMouseDragged:(NSEvent *)event
{
    [self mouseMoved:event];
}

- (void)otherMouseDragged:(NSEvent *)event
{
    [self mouseMoved:event];
}

- (void)mouseEntered:(NSEvent *)event
{
    // set flag and force-release mouse upon entry or exit
    // otherwise mouse interferes with window motion
    mouseInView = 1;
    mouseButton(0, 1, [event locationInWindow]);
}

- (void)mouseExited:(NSEvent *)event
{
    mouseInView = 0;
    mouseButton(0, 1, [event locationInWindow]);
}

- (void)scrollWheel:(NSEvent *)event
{
    // do nothing
}

- (BOOL)performKeyEquivalent:(NSEvent *)event
{
    // this function handles all key presses
    // handle appropriately
    unsigned int key = (unsigned int)[event buttonNumber];
    if (key < 128)
        [self keyDown:event];
    return YES;
}

- (void)flagsChanged:(NSEvent *)event
{
    // do nothing
    // handles special keys
}

- (void)cancelOperation:(id)sender
{
    // do nothing
    // handles cancelled press
}
 

@end
