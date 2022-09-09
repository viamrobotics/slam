
# Temp fix for building grpc, grab minimal rdk and build locally
pull-rdk: bufsetup pull-rdk.sh
	./pull-rdk.sh
	cd rdk-minimal && PATH="${PATH}:`pwd`/../grpc/bin" buf generate --template ../buf.gen.yaml
	cd rdk-minimal && PATH="${PATH}:`pwd`/../grpc/bin" buf generate --template ../buf.grpc.gen.yaml
	cd rdk-minimal && PATH="${PATH}:`pwd`/../grpc/bin" buf generate --template ../buf.gen.yaml buf.build/googleapis/googleapis

bufsetup:
	sudo apt-get install -y protobuf-compiler-grpc libgrpc-dev libgrpc++-dev || brew install grpc openssl --quiet
	GOBIN=`pwd`/grpc/bin go install github.com/bufbuild/buf/cmd/buf@v1.4.0
	ln -sf `which grpc_cpp_plugin` grpc/bin/protoc-gen-grpc-cpp
	pkg-config openssl || export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:`find \`which brew > /dev/null && brew --prefix\` -name openssl.pc | head -n1 | xargs dirname`

clean-all: cleanorb
	rm -rf grpc
	rm -rf rdk-minimal

cleanorb:
	rm -rf viam-orb-slam3/build
	rm -rf viam-orb-slam3/ORB_SLAM3/build
	rm -rf viam-orb-slam3/ORB_SLAM3/Thirdparty/DBoW2/build
	rm -rf viam-orb-slam3/ORB_SLAM3/Thirdparty/g2o/build
	rm -rf viam-orb-slam3/ORB_SLAM3/Thirdparty/Sophus/build
	rm -rf viam-orb-slam3/bin

format:
	find . -type f -not -path \
		-and ! -path '*viam-cartographer/cartographer*' \
		-and ! -path '*viam-cartographer/build*' \
		-and ! -path '*viam-orb-slam3/ORB_SLAM3*' \
		-and ! -path '*rdk-minimal*' \
		-and \( -iname '*.h' -o -iname '*.cpp' -o -iname '*.cc' \) \
		| xargs clang-format -i --style="{BasedOnStyle: Google, IndentWidth: 4}"

buildorb:
	cd viam-orb-slam3 && ./build_orbslam.sh

setuporb: pull-rdk
	cd viam-orb-slam3 && ./setup_orbslam.sh

<<<<<<< HEAD
testorb:
	cd viam-orb-slam3 && ./test_orbslam.sh

orbslam-all: setuporb buildorb testorb


appimage: buildorb
	cd packaging/appimages && appimage-builder --recipe orb_grpc_server-`uname -m`.yml
	mkdir -p packaging/appimages/deploy/
	mv packaging/appimages/*.AppImage* packaging/appimages/deploy/
	chmod 755 packaging/appimages/deploy/*.AppImage
=======


orbslam-all: setuporb pull-rdk buildorb testorb
>>>>>>> 98306e4 (John's comments)

include *.make
