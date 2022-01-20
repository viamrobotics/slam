# SLAM Libraries

## Cartographer

### Installation instructions - OSx

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

# TODO - Make sure these are necessary:
brew install openssl
brew install eigen
brew install gflags
brew install glog
brew install suite-sparse
brew install sphinx-doc
```
 
**Install cartographer**

Run: `./install_cartographer.sh`

Installation tested on:
- [x] 2.4 GHz Quad-Core Intel Core i7; macOS Catalina
- [x] 2.4 GHz 8-Core Intel Core i9; macOS Big Sur
- [x] M1

### Installation instructions - RPI Viam
TODO

## ORB_SLAM3
Download ORB_SLAM3 from here https://github.com/JohnN193/ORB_SLAM3
