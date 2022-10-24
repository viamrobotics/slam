echo "Installing ORB_SLAM3 external dependencies"
brew tap viamrobotics/brews
brew install cmake glew opencv@4 eigen boost pangolin openssl
brew link openssl --force
sudo ln -s /opt/homebrew/Cellar/pangolin/0.8/lib/* /usr/local/lib/
