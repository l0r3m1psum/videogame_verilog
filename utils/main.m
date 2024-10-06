#import <Cocoa/Cocoa.h>
#import <AVFoundation/AVFoundation.h>

@interface AppDelegate : NSObject <NSApplicationDelegate>
@property (strong) NSWindow *window;
@property (strong) AVCaptureDeviceDiscoverySession *discoverySession;
// TODO: try to remove those references.
@property (strong) AVCaptureSession *captureSession;
@property (strong) AVCaptureVideoPreviewLayer *captureVideoPreviewLayer;
@property (strong) NSView *videoView;
@end

// https://stackoverflow.com/questions/67896404/where-is-info-plist-in-xcode-13-missing-not-inside-project-navigator
// TODO: put NSCameraUsageDescription in Info.plist

@implementation AppDelegate

- (void)applicationDidFinishLaunching:(NSNotification *)aNotification {
    NSError *error = nil;
    {
        NSRect frame = NSMakeRect(100, 100, 640, 480);
        NSUInteger styleMask = NSWindowStyleMaskTitled
            | NSWindowStyleMaskClosable
            | NSWindowStyleMaskMiniaturizable;
        self.window = [[NSWindow alloc] initWithContentRect:frame
                                                  styleMask:styleMask
                                                    backing:NSBackingStoreBuffered
                                                      defer:NO];

        [self.window setTitle:@"Viewer"];
        [self.window makeKeyAndOrderFront:nil];
        [self.window center];
    }

    NSMenu *mainMenu = [[NSMenu alloc] initWithTitle:@"MainMenu"];
    [NSApp setMainMenu:mainMenu];

    {
        NSMenuItem *appMenuItem = [[NSMenuItem alloc] init];
        [mainMenu addItem:appMenuItem];
            NSMenu *appMenu = [[NSMenu alloc] initWithTitle:@"App"];
            [appMenuItem setSubmenu:appMenu];

            NSMenuItem *quitMenuItem = [[NSMenuItem alloc] initWithTitle:@"Quit"
                                                                  action:@selector(quitApp:)
                                                           keyEquivalent:@"q"];
            [quitMenuItem setTarget:self];
            [appMenu addItem:quitMenuItem];
    }

    NSMenuItem *devicesMenuItem = [[NSMenuItem alloc] init];
    [mainMenu addItem:devicesMenuItem];
    NSMenu *devicesMenu = [[NSMenu alloc] initWithTitle:@"Devices"];
    [devicesMenuItem setSubmenu:devicesMenu];

    {
        self.videoView = [[NSView alloc] initWithFrame:self.window.contentView.bounds];
        [self.videoView setWantsLayer:YES];
        [self.window.contentView addSubview:self.videoView];
    }

    {
        self.captureSession = [AVCaptureSession new];
        self.captureSession.sessionPreset = AVCaptureSessionPreset640x480;
        self.discoverySession = [AVCaptureDeviceDiscoverySession discoverySessionWithDeviceTypes:@[AVCaptureDeviceTypeExternal]
                                                                                       mediaType:AVMediaTypeVideo
                                                                                        position:AVCaptureDevicePositionUnspecified];

        NSInteger tag = 0;
        for (AVCaptureDevice *device in self.discoverySession.devices) {
			NSString tagString = tag <= 9 ? [@(tag) stringValue] : @"";
            NSMenuItem *menuItem = [[NSMenuItem alloc] initWithTitle:device.localizedName
                                                              action:@selector(deviceSelected:)
                                                       keyEquivalent:tagString];
            menuItem.tag = tag++;
            [menuItem setTarget:self];
            [devicesMenu addItem:menuItem];
			// This is an abomination I know, but it works
            if (tag-1 == 0) [self deviceSelected:menuItem];
        }
    }


    {
        self.captureVideoPreviewLayer = [AVCaptureVideoPreviewLayer layerWithSession:self.captureSession];
        [self.captureVideoPreviewLayer setVideoGravity:AVLayerVideoGravityResizeAspectFill];
        self.captureVideoPreviewLayer.frame = self.videoView.bounds;
        [self.videoView.layer addSublayer:self.captureVideoPreviewLayer];
    }

    [self.captureSession startRunning];
}

- (void)deviceSelected:(id)sender {
    NSError *error = nil;
    [self.captureSession beginConfiguration];

    for (AVCaptureInput *captureInput in self.captureSession.inputs) {
        [self.captureSession removeInput:captureInput];
    }

    NSInteger tag = ((NSMenuItem *)sender).tag;
    AVCaptureDevice *captureDevice = self.discoverySession.devices[tag];

    AVCaptureDeviceInput *captureDeviceInput = [AVCaptureDeviceInput deviceInputWithDevice:captureDevice
                                                                                     error:&error];
    if ([self.captureSession canAddInput:captureDeviceInput]) {
        [self.captureSession addInput:captureDeviceInput];
    }

    [self.captureSession commitConfiguration];
}

- (void)quitApp:(id)sender {
    [NSApp terminate:self];
}

// FIXME: this is not called even with NSSupportsSuddenTermination in Info.plist
// (now it seem to work but it is the selector called by the NSMenu that closes
// the application.)
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
