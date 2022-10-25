echo "Installing ORB_SLAM3 external dependencies"
brew tap viamrobotics/brews
brew install cmake glew opencv@4 eigen boost openssl
export LDFLAGS="$LDFLAGS -L/usr/local/Cellar/openssl@3/3.0.5/lib"
export CPPFLAGS="$CPPFLAGS -I/usr/local/Cellar/openssl@3/3.0.5/include"
export PKG_CONFIG_PATH="$PKG_CONFIG_PATH /usr/local/Cellar/openssl@3/3.0.5/lib/pkgconfig"
brew install pangolin
sudo ln -s /opt/homebrew/Cellar/pangolin/0.8/lib/* /usr/local/lib/ || true
