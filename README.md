# Drop-in Vision-based pose estimation for FRC

I've seen over a decade of folks, including myself, struggle to implement workable computer vision approaches in FRC. In the year 2024, we have access to _great_ off-the-shelf tools in the form of PhotonVision and Limelight. There is no reason for the struggle to get something basic working to continue. This repository implements a single copy-and-paste-into-your-repo approach - simply grab the `com.wcmarshall.dropinlimelight` folder and put it into your `src/main/java` folder.

## What you have to do:

### Code changes

1. Implement the `Chassis` interface in your drive code.
2. Configure the `VisionPoseEstimator::ROBOT_TO_CAMERA` transform using a tape measure or CAD. Or guess and check.
3. Instantiate a `VisionPoseEstimator` using your `Chassis` implementation.
4. Call `VisionPoseEstimator::periodic` every loop.

### LimeLight Setup

1. Plug a USB-C cable into your configuration machine and into the Limelight

#### Imaging

**Always install the latest firmware**

Follow the directions in the Limelight [website](https://docs.limelightvision.io/docs/docs-limelight/getting-started/FRC/imaging). If you only have access to a linux machine, install `rpiboot` and use that to update the image. See directions [here](https://robo.fish/wiki/index.php?title=Raspberry_Pi#Installing_the_System_Image_on_eMMC) for other CM-based devices.

#### Configuration

1. Navigate to http://limelight.local:5801/
2. Set team number in the settings, save and power cycle
3. Configure automatic or static addressing (I prefer automatic, LimeLight recommends static)
4. Configure the 0th pipeline for apriltags, following the [docs](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-3d#full-3d-tracking)
5. Upload a field map, such as the example one [here](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-map-specification)