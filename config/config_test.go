package config

import (
	"testing"

	"github.com/edaniels/golog"
	"go.viam.com/test"
)

var (
	_true  = true
	_false = false
)

func TestDetermineDeleteProcessedData(t *testing.T) {

	logger := golog.NewTestLogger(t)

	t.Run("No delete_processed_data provided", func(t *testing.T) {
		deleteProcessedData := DetermineDeleteProcessedData(logger, nil, false)
		test.That(t, deleteProcessedData, test.ShouldBeFalse)

		deleteProcessedData = DetermineDeleteProcessedData(logger, nil, true)
		test.That(t, deleteProcessedData, test.ShouldBeTrue)
	})

	t.Run("False delete_processed_data", func(t *testing.T) {
		deleteProcessedData := DetermineDeleteProcessedData(logger, &_false, false)
		test.That(t, deleteProcessedData, test.ShouldBeFalse)

		deleteProcessedData = DetermineDeleteProcessedData(logger, &_false, true)
		test.That(t, deleteProcessedData, test.ShouldBeFalse)
	})

	t.Run("True delete_processed_data", func(t *testing.T) {
		deleteProcessedData := DetermineDeleteProcessedData(logger, &_true, false)
		test.That(t, deleteProcessedData, test.ShouldBeFalse)

		deleteProcessedData = DetermineDeleteProcessedData(logger, &_true, true)
		test.That(t, deleteProcessedData, test.ShouldBeTrue)
	})
}

func TestDetermineUseLiveData(t *testing.T) {

	logger := golog.NewTestLogger(t)
	t.Run("No use_live_data specified", func(t *testing.T) {
		useLiveData, err := DetermineUseLiveData(logger, nil, []string{})
		test.That(t, err, test.ShouldBeError, ConfigError("use_live_data is a required input parameter"))
		test.That(t, useLiveData, test.ShouldBeFalse)

		useLiveData, err = DetermineUseLiveData(logger, nil, []string{"camera"})
		test.That(t, err, test.ShouldBeError, ConfigError("use_live_data is a required input parameter"))
		test.That(t, useLiveData, test.ShouldBeFalse)
	})
	t.Run("False use_live_data", func(t *testing.T) {
		useLiveData, err := DetermineUseLiveData(logger, &_false, []string{})
		test.That(t, err, test.ShouldBeNil)
		test.That(t, useLiveData, test.ShouldBeFalse)

		useLiveData, err = DetermineUseLiveData(logger, &_false, []string{"camera"})
		test.That(t, err, test.ShouldBeNil)
		test.That(t, useLiveData, test.ShouldBeFalse)
	})
	t.Run("True use_live_data", func(t *testing.T) {
		useLiveData, err := DetermineUseLiveData(logger, &_true, []string{})
		test.That(t, err, test.ShouldBeError, ConfigError("sensors field cannot be empty when use_live_data is set to true"))
		test.That(t, useLiveData, test.ShouldBeFalse)

		useLiveData, err = DetermineUseLiveData(logger, &_true, []string{"camera"})
		test.That(t, err, test.ShouldBeNil)
		test.That(t, useLiveData, test.ShouldBeTrue)
	})
}
