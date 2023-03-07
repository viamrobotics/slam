package config

import (
	"context"
	"os"
	"path/filepath"
	"time"

	"github.com/edaniels/golog"
	"github.com/pkg/errors"
	"go.opencensus.io/trace"
	pb "go.viam.com/api/service/slam/v1"
	"google.golang.org/grpc"
	"google.golang.org/grpc/credentials/insecure"
)

// SetupDirectories creates the core data, map, and config directories at the end of the passed path.
func SetupDirectories(dataDirectory string, logger golog.Logger) error {
	for _, directoryName := range [4]string{"", "data", "map", "config"} {
		directoryPath := filepath.Join(dataDirectory, directoryName)
		if _, err := os.Stat(directoryPath); os.IsNotExist(err) {
			logger.Warnf("%v directory does not exist", directoryPath)
			if err := os.Mkdir(directoryPath, os.ModePerm); err != nil {
				return errors.Errorf("issue creating directory at %v: %v", directoryPath, err)
			}
		}
	}
	return nil
}

// SetupGRPCConnection uses the defined port to create a GRPC client for communicating with the SLAM algorithms.
func SetupGRPCConnection(
	ctx context.Context,
	port string,
	dialMaxTimeoutSec int,
	logger golog.Logger,
) (pb.SLAMServiceClient, func() error, error) {
	ctx, span := trace.StartSpan(ctx, "slam::builtIn::setupGRPCConnection")
	defer span.End()

	// This takes about 1 second, so the timeout should be sufficient.
	ctx, timeoutCancel := context.WithTimeout(ctx, time.Duration(dialMaxTimeoutSec)*time.Second)
	defer timeoutCancel()
	// The 'port' provided in the config is already expected to include "localhost:", if needed, so that it doesn't need to be
	// added anywhere in the code. This will allow cloud-based SLAM processing to exist in the future.
	// TODO: add credentials when running SLAM processing in the cloud.

	// Increasing the gRPC max message size from the default value of 4MB to 32MB, to match the limit that is set in RDK. This is
	// necessary for transmitting large pointclouds.
	maxMsgSizeOption := grpc.WithDefaultCallOptions(grpc.MaxCallRecvMsgSize(32 * 1024 * 1024))
	connLib, err := grpc.DialContext(ctx, port, grpc.WithTransportCredentials(insecure.NewCredentials()), grpc.WithBlock(), maxMsgSizeOption)
	if err != nil {
		logger.Errorw("error connecting to slam process", "error", err)
		return nil, nil, err
	}
	return pb.NewSLAMServiceClient(connLib), connLib.Close, err
}
