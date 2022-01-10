# Cartographer

## Installation notes

### Kat's steps
* brew update
* brew upgrade

```bash
clang --version
Apple clang version 13.0.0 (clang-1300.0.29.30)
Target: x86_64-apple-darwin20.6.0
Thread model: posix
InstalledDir: /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin
```

```bash
cmake --version
cmake version 3.22.1
CMake suite maintained and supported by Kitware (kitware.com/cmake).
```

```bash
g++ --version
Configured with: --prefix=/Applications/Xcode.app/Contents/Developer/usr --with-gxx-include-dir=/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk/usr/include/c++/4.2.1
Apple clang version 13.0.0 (clang-1300.0.29.30)
Target: x86_64-apple-darwin20.6.0
Thread model: posix
InstalledDir: /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin
```

```bash
git --version 
git version 2.32.0 (Apple Git-132)
```

Assuming I can do these (based on my best guess):
* google-mock --> `brew install googletest`
* libboost-all-dev --> `brew install boost`
* libcairo2-dev --> `brew install cairo`
* libcurl4-openssl-dev --> `brew install openssl`
* libeigen3-dev -->  `brew install eigen`
* libgflags-dev --> `brew install gflags`
* libgoogle-glog-dev --> `brew install glog`
* liblua5.2-dev --> `brew install lua`
* libsuitesparse-dev --> `brew install suite-sparse`
* lsb-release --> LINUX thing
* ninja-build --> `brew install ninja`
* stow --> `brew install stow`

* python3-sphinx --> `brew install sphinx-doc`
* libgmock-dev --> Assuming it's already installed via `brew install googletest` from above
* libceres-dev --> `brew install ceres-solver`
* protobuf-compiler --> `brew install protobuf && brew uprade protobuf`


Complained about LuaGoogle not found; went into CMakeLists to change LuaGoogle --> Lua.


Fails to link ; am doing the following:
* Uninstall `brew uninstall abseil`, installing it by following the instructions [here](https://google-cartographer.readthedocs.io/en/latest/), like this:

```bash
git clone https://github.com/abseil/abseil-cpp.git
cd abseil-cpp
git checkout d902eb869bcfacc1bad14933ed9af4bed006d481
mkdir build
cd build
cmake -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DCMAKE_INSTALL_PREFIX=/usr/local/stow/absl \
  ..
ninja
sudo ninja install
cd /usr/local/stow
sudo stow absl
```


Running this: `CTEST_OUTPUT_ON_FAILURE=1 ninja test` resulted in two failed tests, which I ignored.

Running this: `sudo ninja install` was successful!