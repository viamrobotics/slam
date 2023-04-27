package lidar_test

import (
	"testing"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/registry"
	"go.viam.com/slam/sensors/lidar"
	"go.viam.com/test"
)

const (
	testLidarName  = "testLidarName"
	wrongLidarName = "wrongLidarName"
)

// Test:
// New function: empty dependencies, wrong name, successful creation of Lidar
// GetData: get next pointcloud

type mockLidar struct {
	camera.Camera
	Name string
}

func TestNew(t *testing.T) {
	t.Run("Empty dependencies empty sensors failure", func(t *testing.T) {
		deps := make(registry.Dependencies)
		sensors := []string{}
		lidar, err := lidar.New(deps, sensors, 0)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, lidar, test.ShouldNotBeNil)
	})

	t.Run("Empty dependencies non-empty sensors failure", func(t *testing.T) {
		deps := make(registry.Dependencies)
		sensors := []string{testLidarName}
		lidar, err := lidar.New(deps, sensors, 0)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, lidar, test.ShouldNotBeNil)
	})

	t.Run("Wrong sensor name failure", func(t *testing.T) {
		deps := make(registry.Dependencies)
		deps[camera.Named(testLidarName)] = &mockLidar{Name: testLidarName}
		sensors := []string{wrongLidarName}
		lidar, err := lidar.New(deps, sensors, 0)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, lidar, test.ShouldNotBeNil)
	})

}
