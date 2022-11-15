# SLAM Libraries

## (In)stability Notice
> **Warning**
> This is an experimental feature. Stability is not guaranteed. Breaking changes are likely to occur, and occur often.

## Overview
The following is an overview over the SLAM libraries that have been integrated so far.

To use these servers the Viam API must be built.
To setup, run:

```bash
make bufinstall buf
```

## Cartographer
[Cartographer](https://github.com/cartographer-project/cartographer) is a system that provides real-time simultaneous localization
and mapping [SLAM](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) in 2D and 3D across multiple platforms and sensor
configurations.

Learn more at [the official Docs site](https://google-cartographer.readthedocs.io).

### Getting started
To automatically install dependencies run
```
make setupcarto
```

To build cartographer use
```
make buildcarto
```
Then, follow the instruction as described in [viam-cartographer/README.md](./viam-cartographer/README.md).

## ORB_SLAM3
ORB_SLAM3 is a SLAM system for feature based mapping using monocular, rgbd, and stereo camera setups.

For more information see [the official ORB_SLAM3 Repo](https://github.com/UZ-SLAMLab/ORB_SLAM3).

### Getting started

To automatically install dependencies run
```
make setuporb
```

To build ORB_SLAM3 and thirdparty packages use
```
make buildorb
```

Then, follow the instruction as described in [viam-orb-slam3/README.md](./viam-orb-slam3/README.md).

