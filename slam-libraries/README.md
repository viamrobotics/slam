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
brew install googletest
brew install boost
brew install cairo
brew install openssl
brew install eigen
brew install gflags
brew install glog
brew install suite-sparse
brew install ninja
brew install sphinx-doc
```

```bash
curl -o lua-5.2.4.tar.gz https://www.lua.org/ftp/lua-5.2.4.tar.gz
tar -xvf lua-5.2.4.tar.gz
pushd lua-5.2.4
make macosx
sudo make install
popd
rm lua-5.2.4.tar.gz
rm -rf lua-5.2.4
```

**Install cartographer**

Run: `./install_cartographer.sh`

Installation tested on:
- [x] 2.4 GHz Quad-Core Intel Core i7; macOS Catalina
- [x] 2.4 GHz 8-Core Intel Core i9; macOS Big Sur
- [x] M1


## ORB_SLAM3
Download ORB_SLAM3 from here https://github.com/JohnN193/ORB_SLAM3
