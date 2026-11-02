## NOTICE

This repository contains the public FTC SDK for the BIOBUZZ (2026-2027) competition season.

FTC Robot Code

This repo is the code for our FTC robot. It's built on the FTC SDK and uses Pedro Pathing for path following.

What's in TeamCode

The robot code lives in the TeamCode module. It has the hardware setup, the intake, and two TeleOps, robot centric and field centric. Field centric driving uses the heading from a goBILDA Pinpoint.

Driver input goes through a custom joystick response curve with a smooth fillet between zones, and an acceleration limiter that only limits speeding up, never direction. To keep the loop fast, the code skips motor commands that barely changed and only reads the Pinpoint's heading in TeleOp.

Status

TeleOp is working. Autonomous is still in progress.

Credits

Built on the [FTC SDK](https://github.com/FIRST-Tech-Challenge/FtcRobotController) by FIRST and [Pedro Pathing](https://pedropathing.com/), each under its own license in this repo.
