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
