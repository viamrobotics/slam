echo "Installing ORB_SLAM3 external dependencies"
brew tap viamrobotics/brews
brew install cmake glew opencv@4 eigen boost pangolin openssl
export LDFLAGS="-L/usr/local/opt/openssl@3/lib"
export CPPFLAGS="-I/usr/local/opt/openssl@3/include"
sudo ln -s /opt/homebrew/Cellar/pangolin/0.8/lib/* /usr/local/lib/
