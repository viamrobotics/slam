echo "Installing ORB_SLAM3 external dependencies"
brew tap viamrobotics/brews
brew install cmake glew opencv@4 eigen boost openssl
sudo ln -s /usr/local/opt/openssl/lib/libcrypto.1.0.0.dylib /usr/local/lib/
sudo ln -s /usr/local/opt/openssl/lib/libssl.1.0.0.dylib /usr/local/lib/
sudo ln -s /usr/local/Cellar/openssl/1.0.2j/bin/openssl /usr/local/bin/openssl
brew install pangolin
sudo ln -s /opt/homebrew/Cellar/pangolin/0.8/lib/* /usr/local/lib/
