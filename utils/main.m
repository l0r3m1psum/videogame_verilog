#import <Cocoa/Cocoa.h>
#import <AVFoundation/AVFoundation.h>

@interface AppDelegate : NSObject <NSApplicationDelegate>
@property (strong) NSWindow *window;
// TODO: try to remove those references.
@property (strong) AVCaptureSession *captureSession;
@property (strong) AVCaptureVideoPreviewLayer *captureVideoPreviewLayer;
@property (strong) NSView *videoView;
@end

// https://stackoverflow.com/questions/67896404/where-is-info-plist-in-xcode-13-missing-not-inside-project-navigator
// TODO: put NSCameraUsageDescription in Info.plist

@implementation AppDelegate

- (void)applicationDidFinishLaunching:(NSNotification *)aNotification {
    {
        NSRect frame = NSMakeRect(100, 100, 640, 480);
        NSUInteger styleMask = NSWindowStyleMaskTitled
            | NSWindowStyleMaskClosable
            | NSWindowStyleMaskMiniaturizable;
        self.window = [[NSWindow alloc] initWithContentRect:frame
                                                  styleMask:styleMask
                                                    backing:NSBackingStoreBuffered
                                                      defer:NO];

        [self.window setTitle:@"My Programmatic Window"];
        [self.window makeKeyAndOrderFront:nil];
        [self.window center];
    }

    {
        self.videoView = [[NSView alloc] initWithFrame:self.window.contentView.bounds];
        [self.videoView setWantsLayer:YES];
        [self.window.contentView addSubview:self.videoView];
    }

    {
        NSError *error = nil;
        self.captureSession = [AVCaptureSession new];
        self.captureSession.sessionPreset = AVCaptureSessionPreset640x480;
        AVCaptureDeviceDiscoverySession *discoverySession = nil;
        discoverySession = [AVCaptureDeviceDiscoverySession discoverySessionWithDeviceTypes:@[AVCaptureDeviceTypeExternal]
                                                                                  mediaType:AVMediaTypeVideo
                                                                                   position:AVCaptureDevicePositionUnspecified];
        AVCaptureDevice *captureDevice = discoverySession.devices.firstObject;
        AVCaptureDeviceInput *captureDeviceInput = [AVCaptureDeviceInput deviceInputWithDevice:captureDevice
                                                                                         error:&error];
        if ([self.captureSession canAddInput:captureDeviceInput]) {
            [self.captureSession addInput:captureDeviceInput];
        } else NSLog(@"can't add input");
    }

    {
        self.captureVideoPreviewLayer = [AVCaptureVideoPreviewLayer layerWithSession:self.captureSession];
        [self.captureVideoPreviewLayer setVideoGravity:AVLayerVideoGravityResizeAspectFill];
        self.captureVideoPreviewLayer.frame = self.videoView.bounds;
        [self.videoView.layer addSublayer:self.captureVideoPreviewLayer];
    }

    [self.captureSession startRunning];
}

// FIXME: this is not called even with NSSupportsSuddenTermination in Info.plist
- (NSApplicationTerminateReply)applicationShouldTerminate:(NSApplication *)app {
    return NSTerminateNow;
}

- (BOOL)applicationShouldTerminateAfterLastWindowClosed:(NSApplication *)app {
     return YES;
}

- (void)applicationWillTerminate:(NSNotification *)aNotification {
}

- (BOOL)applicationSupportsSecureRestorableState:(NSApplication *)app {
    return YES;
}

@end

int main(int argc, const char * argv[]) {
    @autoreleasepool {
        NSApplication *app = [NSApplication sharedApplication];
        AppDelegate *delegate = [AppDelegate new];
        [app setDelegate:delegate];
        [app run];
    }
    return EXIT_SUCCESS;
}
