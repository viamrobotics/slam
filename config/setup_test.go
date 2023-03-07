package config

import (
	"context"
	"os"
	"testing"

	"github.com/edaniels/golog"
	"github.com/pkg/errors"
	"go.opencensus.io/trace"
	"go.viam.com/test"
)

func TestGRPCConnection(t *testing.T) {
	logger := golog.NewTestLogger(t)
	t.Run("Invalid grpc connection", func(t *testing.T) {
		ctx, _ := trace.StartSpan(context.Background(), "slam::test::TestGRPCConnection")
		port := "invalid_unused_port:0"
		_, _, err := SetupGRPCConnection(ctx, port, 1, logger)
		test.That(t, err, test.ShouldBeError, errors.New("context deadline exceeded"))
	})
}

func TestSetupDirectories(t *testing.T) {
	logger := golog.NewTestLogger(t)
	tempDir, err := os.MkdirTemp("", "*")
	defer os.RemoveAll(tempDir)
	test.That(t, err, test.ShouldBeNil)
	SetupDirectories(tempDir, logger)
	// Ensure that all of the directories have been created
	_, errData := os.Stat(tempDir + "/data")
	test.That(t, errData, test.ShouldBeNil)
	_, errMap := os.Stat(tempDir + "/map")
	test.That(t, errMap, test.ShouldBeNil)
	_, errConfig := os.Stat(tempDir + "/config")
	test.That(t, errConfig, test.ShouldBeNil)
	// Ensure that the tests work
	_, errFoo := os.Stat(tempDir + "/foodir")
	test.That(t, errFoo, test.ShouldBeError)
}
