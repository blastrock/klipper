# Axis Twist Compensation

This document describes the [axis_twist_compensation] module.

Some printers exhibit different Z offsets depending on the X position of the
probed point, and this is independent of the bed being not flat or not trammed.
This is common in printers with designs like the Prusa MK3, Sovol SV06 etc and is
further described under [probe location
bias](Probe_Calibrate.md#location-bias-check). It may result in
probe operations such as [Bed Mesh](Bed_Mesh.md),
[Screws Tilt Adjust](G-Codes.md#screws_tilt_adjust),
[Z Tilt Adjust](G-Codes.md#z_tilt_adjust) etc returning inaccurate
representations of the bed.

This module uses manual measurements by the user to correct the probe's results.
Note that if your axis is significantly twisted it is strongly recommended to
first use mechanical means to fix it prior to applying software corrections.

## Overview of compensation usage

> **Tip:** Make sure the [probe X and Y offsets](Config_Reference.md#probe) are
> correctly set as they greatly influence calibration.

> **Tip:** A feeler gauge is recommended in place of paper for more accurate
> measurements

1. After setting up the [axis_twist_compensation] module,
perform `AXIS_TWIST_COMPENSATION_CALIBRATE`
* The calibration wizard will prompt you to measure the probe Z offset at a few
points along the bed
* The calibration defaults to 3 points but you can use the option `N_POINTS=` to
use a different number.
2. [Adjust your Z offset](Probe_Calibrate.md#calibrating-probe-z-offset)
3. Perform automatic/probe-based bed tramming operations, such as
[Screws Tilt Adjust](G-Codes.md#screws_tilt_adjust),
[Z Tilt Adjust](G-Codes.md#z_tilt_adjust) etc
4. Home all axis, then perform a [Bed Mesh](Bed_Mesh.md) if required
5. Perform a test print, followed by any
[fine-tuning](Axis_Twist_Compensation.md#fine-tuning) as desired

> **Tip:** Bed temperature and nozzle temperature and size do not seem to have
> an influence to the calibration process.

> **Tip:** Disable the Axis twist compensation using `AXIS_TWIST_PROFILE_CLEAR`
> and perform Screws Tilt Adjust/Z Tilt Adjust/Bed Mesh to compare the
> difference

## [axis_twist_compensation] setup and commands

Configuration options for [axis_twist_compensation] can be found in the
[Configuration Reference](Config_Reference.md#axis_twist_compensation).

Commands for [axis_twist_compensation] can be found in the
[G-Codes Reference](G-Codes.md#axis_twist_compensation)

### Type of compensation

This module supports two interpolation methods for the Z offset compensation:
linear and multilinear.

Some printers which have a slight twist only require a linear compensation, so
you can use the 'linear' mode to do a linear regression to fit a line through
the measured compensation.

It is not uncommon for printers to have multiple combined non-linear twists
(e.g. the tool head leaning forward in the middle of the axis due to its own
weight). For that, it is recommended to use the 'multilinear' mode which will
interpolate linearly between each pair of points.

> **Tip:** It is not necessary to redo a calibration when switching between
> linear and multilinear.

### Fine-tuning

If required, the user may fine tune the `z_compensations` values.

For example, the value of `0.12` represents the `z_compensation` applied
when the printer is probing at `start_x`. This means that if the non-compensated
z-height measured at `start_x` was `2.0`, the compensated z-height is now `2.0 +
0.12 = 2.12`, resulting in a higher point on the recorded mesh.

Should the squish on the side of the bed represented by `start_x` be
insufficient, the user may specify a lower `z_compensation` value in place of
`0.12`, such as `0.02`. This would result in the mesh being lowered by
`0.12 - 0.02 = 0.1mm` at points along `start_x`.

## Reference

The calculations for this module are largely based off those found in
[Marlin PR #23238](https://github.com/MarlinFirmware/Marlin/pull/23238).

A visualization of the issue can be found in
[this discussion thread](https://github.com/MarlinFirmware/Marlin/issues/22791).
