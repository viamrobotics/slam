echo "Installing ORB_SLAM3 external dependencies"
brew tap viamrobotics/brews
brew install cmake glew opencv@4 eigen boost openssl
LDFLAGS=-L/usr/local/opt/openssl@3/lib
CPPFLAGS=-I/usr/local/opt/openssl@3/include
PKG_CONFIG_PATH=/usr/local/opt/openssl@3/lib/pkgconfig
brew install pangolin
sudo ln -s /opt/homebrew/Cellar/pangolin/0.8/lib/* /usr/local/lib/ || true
