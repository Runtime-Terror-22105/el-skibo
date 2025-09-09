# RTT DECODE Code
This is the repository for Runtime Terror's code for the DECODE 2025-2026 season for the First Tech Challenge.

## Requirements
Make sure you're using Android Studio Ladybug (2022.3.1) or newer.

## Units
In this project, the following units are always expected to be used:
- Time -> Milliseconds
- Angles -> Radians
- Distances -> Inches, except for pinpoint which is mm

# Recent Update - Version 11.0
## Enhancements

* Adds support for AndyMark ToF, IMU, and Color sensors.
* Adds several features to the Color Processing software:
  * DECODE colors `ARTIFACT_GREEN` and `ARTIFACT_PURPLE`
  * Choice of the order of pre-processing steps Erode and Dilate
  * Best-fit preview shape called `circleFit`, an alternate to the existing `boxFit`
  * Sample OpMode `ConceptVisionColorLocator_Circle`, an alternate to the renamed `ConceptVisionColorLocator_Rectangle`
* The Driver Station app play button has a green background with a white play symbol if
  * the driver station and robot controller are connected and have the same team number
  * there is at least one gamepad attached
  * the timer is enabled (for an Autonomous OpMode)
* Updated AprilTag Library for DECODE. Notably, getCurrentGameTagLibrary() now returns DECODE tags.
  * Since the AprilTags on the Obelisk should not be used for localization, the ConceptAprilTagLocalization samples only use those tags without the name 'Obelisk' in them.
* OctoQuad I2C driver updated to support firmware v3.x 
  * Adds support for odometry localizer on MK2 hardware revision
  * Adds ability to track position for an absolute encoder across multiple rotations
  * Note that some driver APIs have changed; minor updates to user software may be required
  * Requires firmware v3.x. For instructions on updating firmware, see
    https://github.com/DigitalChickenLabs/OctoQuad/blob/master/documentation/OctoQuadDatasheet_Rev_3.0C.pdf

# Getting Help
## User Documentation and Tutorials
*FIRST* maintains online documentation with information and tutorials on how to use the *FIRST* Tech Challenge software and robot control system.  You can access this documentation using the following link:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FIRST Tech Challenge Documentation](https://ftc-docs.firstinspires.org/index.html)

Note that the online documentation is an "evergreen" document that is constantly being updated and edited.  It contains the most current information about the *FIRST* Tech Challenge software and control system.

## Javadoc Reference Material
The Javadoc reference documentation for the FTC SDK is now available online.  Click on the following link to view the FTC SDK Javadoc documentation as a live website:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FTC Javadoc Documentation](https://javadoc.io/doc/org.firstinspires.ftc)

## Online User Forum
For technical questions regarding the Control System or the FTC SDK, please visit the FIRST Tech Challenge Community site:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FIRST Tech Challenge Community](https://ftc-community.firstinspires.org/)

## Sample OpModes
This project contains a large selection of Sample OpModes (robot code examples) which can be cut and pasted into your /teamcode folder to be used as-is, or modified to suit your team's needs.

Samples Folder: &nbsp;&nbsp; [/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples](FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples)

The readme.md file located in the [/TeamCode/src/main/java/org/firstinspires/ftc/teamcode](TeamCode/src/main/java/org/firstinspires/ftc/teamcode) folder contains an explanation of the sample naming convention, and instructions on how to copy them to your own project space.
