// Package config implements functions to assist with attribute evaluation in the slam service
package config

import (
	"fmt"

	"github.com/edaniels/golog"
	"github.com/pkg/errors"
	"go.viam.com/rdk/config"
	"go.viam.com/rdk/registry"
	"go.viam.com/utils"
)

var (
	cameraValidationMaxTimeoutSec = 30 // reconfigurable for testing
	dialMaxTimeoutSec             = 30 // reconfigurable for testing
)

const (
	defaultDataRateMs           = 200
	minDataRateMs               = 200
	defaultMapRateSec           = 60
	cameraValidationIntervalSec = 1.
	parsePortMaxTimeoutSec      = 60
	// time format for the slam service.
	slamTimeFormat        = "2006-01-02T15:04:05.0000Z"
	opTimeoutErrorMessage = "bad scan: OpTimeout"
	localhost0            = "localhost:0"
)

// SetCameraValidationMaxTimeoutSecForTesting sets cameraValidationMaxTimeoutSec for testing.
func SetCameraValidationMaxTimeoutSecForTesting(val int) {
	cameraValidationMaxTimeoutSec = val
}

// SetDialMaxTimeoutSecForTesting sets dialMaxTimeoutSec for testing.
func SetDialMaxTimeoutSecForTesting(val int) {
	dialMaxTimeoutSec = val
}

// NewError returns an error specific to a failure in the SLAM config.
func NewError(configError string) error {
	return errors.Errorf("SLAM Service configuration error: %s", configError)
}

// DetermineDeleteProcessedData will determine the value of the deleteProcessData attribute
// based on the useLiveData and deleteData input parameters.
func DetermineDeleteProcessedData(logger golog.Logger, deleteData *bool, useLiveData bool) bool {
	var deleteProcessedData bool
	if deleteData == nil {
		deleteProcessedData = useLiveData
	} else {
		deleteProcessedData = *deleteData
		if !useLiveData && deleteProcessedData {
			logger.Debug("a value of true cannot be given for delete_processed_data when in offline mode, setting to false")
			deleteProcessedData = false
		}
	}
	return deleteProcessedData
}

// DetermineUseLiveData will determine the value of the useLiveData attribute
// based on the liveData input parameter and sensor list.
func DetermineUseLiveData(logger golog.Logger, liveData *bool, sensors []string) (bool, error) {
	if liveData == nil {
		return false, NewError("use_live_data is a required input parameter")
	}
	useLiveData := *liveData
	if useLiveData && len(sensors) == 0 {
		return false, NewError("sensors field cannot be empty when use_live_data is set to true")
	}
	return useLiveData, nil
}

// AttrConfig describes how to configure the service.
type AttrConfig struct {
	Sensors             []string          `json:"sensors"`
	ConfigParams        map[string]string `json:"config_params"`
	DataDirectory       string            `json:"data_dir"`
	UseLiveData         *bool             `json:"use_live_data"`
	DataRateMs          int               `json:"data_rate_msec"`
	MapRateSec          *int              `json:"map_rate_sec"`
	InputFilePattern    string            `json:"input_file_pattern"`
	Port                string            `json:"port"`
	DeleteProcessedData *bool             `json:"delete_processed_data"`
	Dev                 bool              `json:"dev"`
}

func NewAttrConfig(cfg config.Service, deps registry.Dependencies) (*AttrConfig, error) {
	attrCfg := &AttrConfig{
		Sensors:             cfg.Attributes.StringSlice("sensors"),
		ConfigParams:        make(map[string]string),
		DataDirectory:       cfg.Attributes.String("data_dir"),
		UseLiveData:         nil,
		DataRateMs:          cfg.Attributes.Int("data_rate_msec", 0),
		MapRateSec:          nil,
		Port:                cfg.Attributes.String("port"),
		DeleteProcessedData: nil,
		Dev:                 cfg.Attributes.Bool("dev", false),
	}


	if cfg.Attributes.Has("use_live_data") {
        useLive := cfg.Attributes.Bool("use_live_data",false);
        attrCfg.UseLiveData = &useLive;
	}

	if cfg.Attributes.Has("delete_processed_data") {
        deleteProcessed := cfg.Attributes.Bool("delete_processed_data", false);
        attrCfg.DeleteProcessedData = &deleteProcessed;
	}

	if cfg.Attributes.Has("map_rate_sec") {
		mapRateSec := cfg.Attributes.Int("map_rate_sec", 0)
		attrCfg.MapRateSec = &mapRateSec
	}

    configParams := cfg.Attributes["config_params"]
    if configParams != nil {
        if config, ok := configParams.(map[string]interface{}); ok {
            for k, valueInterface := range config {
                if valueString, ok := valueInterface.(string); ok {
                    attrCfg.ConfigParams[k] = valueString
                }else {
                    return nil, NewError(fmt.Sprintf("unable to load config_params. Key (%s) must have a string value", k))
                }
            }
        } else {
            return nil, NewError("unable to load config_params. It should be a map of strings to strings.")
        }
        
    }

	_, err := attrCfg.Validate("TODO")
	if err != nil {
		return &AttrConfig{}, err
	}

	return attrCfg, nil
}

// Validate creates the list of implicit dependencies.
func (config *AttrConfig) Validate(path string) ([]string, error) {

	if config.ConfigParams["mode"] == "" {
		return nil, utils.NewConfigValidationFieldRequiredError(path, "config_params[mode]")
	}

	if config.DataDirectory == "" {
		return nil, utils.NewConfigValidationFieldRequiredError(path, "data_dir")
	}

	if config.UseLiveData == nil {
		return nil, utils.NewConfigValidationFieldRequiredError(path, "use_live_data")
	}

	if config.DataRateMs != 0 && config.DataRateMs < minDataRateMs {
		return nil, errors.Errorf("cannot specify data_rate_msec less than %v", minDataRateMs)
	}

	if config.MapRateSec != nil && *config.MapRateSec < 0 {
		return nil, errors.New("cannot specify map_rate_sec less than zero")
	}

	deps := config.Sensors

	return deps, nil
}


// SetParameters ...
func (config *AttrConfig) SetParameters(localhost0 string, defaultDataRateMs, defaultMapRateSec int, logger golog.Logger) error {

	if config.Port == "" {
		config.Port = localhost0
	}

	if config.DataRateMs == 0 {
		config.DataRateMs = defaultDataRateMs
		logger.Debugf("no data_rate_msec given, setting to default value of %d", defaultDataRateMs)
	}

	if config.MapRateSec == nil {
		logger.Debugf("no map_rate_secs given, setting to default value of %d", defaultMapRateSec)
		config.MapRateSec = &defaultMapRateSec
	} else if *config.MapRateSec == 0 {
		logger.Info("setting slam system to localization mode")
	}

	useLiveData, err := DetermineUseLiveData(logger, config.UseLiveData, config.Sensors)
	if err != nil {
		return err
	}
	config.UseLiveData = &useLiveData

	deleteProcessedData := DetermineDeleteProcessedData(logger, config.DeleteProcessedData, useLiveData)
	config.DeleteProcessedData = &deleteProcessedData
	return nil
}
