# Cartographer

[Cartographer](https://github.com/cartographer-project/cartographer) is a system that provides real-time simultaneous localization
and mapping [SLAM](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) in 2D and 3D across multiple platforms and sensor
configurations.


Learn more at [the official Docs site](https://google-cartographer.readthedocs.io).


## Installation instructions

### OSx

**Install Xcode**
1. Install/Update Xcode from App Store
1. Install Xcode cmdline tools: `xcode-select --install`

**Install dependencies**
```bash
# Update and upgrade Brew
brew update
brew upgrade
```

```bash
# Install dependencies
brew install abseil boost ceres-solver protobuf ninja cairo googletest lua@5.3
brew link lua@5.3

# TODO[kat]: Make sure these are necessary:
brew install openssl
brew install eigen
brew install gflags
brew install glog
brew install suite-sparse
brew install sphinx-doc
```
 
**Build cartographer**

Run: `./scripts/build_cartographer.sh`

Installation & building tested on:
- [ ] 2.4 GHz Quad-Core Intel Core i7; macOS Catalina
- [ ] 2.4 GHz 8-Core Intel Core i9; macOS Big Sur
- [ ] M1

This needs to be built only once.

**Build viam-cartographer**

Run: `./scripts/build_viam_cartographer.sh`

Installation & building tested on:
- [ ] 2.4 GHz Quad-Core Intel Core i7; macOS Catalina
- [ ] 2.4 GHz 8-Core Intel Core i9; macOS Big Sur
- [ ] M1

This is run frequently, as this is where we're building our code.

A second build script is therefore provided for local development: `./scripts/rebuild_viam_cartographer.sh`.

### [TODO: needs update] Raspberry Pi Viam Image (Or other Debian 11+, including canon-shell)
TODO[kat]: Make sure this is correct & works - haven't tested in a long time.
NOTE[05/31/22]: This has not been tested with the refactored code, where the main
cartographer repo was split from the viam-cartographer wrapper code.

```bash
sudo apt install cmake \
ninja-build \
libgmock-dev \
libboost-iostreams-dev \
liblua5.3-dev \
libcairo2-dev \
python3-sphinx \
libabsl-dev \
libceres-dev \
libprotobuf-dev \
protobuf-compiler \
libpcl-dev
```

## Running cartographer
Configure how to run cartographer in this file: [scripts/run_cartographer.sh](./scripts/run_cartographer.sh).

Run cartographer by executing: `./scripts/run_cartographer.sh`
