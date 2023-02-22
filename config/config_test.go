package config

import (
	"testing"
    "context"

	"github.com/edaniels/golog"
	"go.viam.com/rdk/config"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/registry"
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
		test.That(t, err, test.ShouldBeError, NewError("use_live_data is a required input parameter"))
		test.That(t, useLiveData, test.ShouldBeFalse)

		useLiveData, err = DetermineUseLiveData(logger, nil, []string{"camera"})
		test.That(t, err, test.ShouldBeError, NewError("use_live_data is a required input parameter"))
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
		test.That(t, err, test.ShouldBeError, NewError("sensors field cannot be empty when use_live_data is set to true"))
		test.That(t, useLiveData, test.ShouldBeFalse)

		useLiveData, err = DetermineUseLiveData(logger, &_true, []string{"camera"})
		test.That(t, err, test.ShouldBeNil)
		test.That(t, useLiveData, test.ShouldBeTrue)
	})
}


func createSLAMService(
	t *testing.T,
	attrCfg *AttrConfig,
	model string,
	logger golog.Logger,
	bufferSLAMProcessLogs bool,
	success bool,
) (config.Service, error) {
	t.Helper()

	ctx := context.Background()
	cfgService := config.Service{Name: "test", Type: "slam", Model: resource.NewDefaultModel(resource.ModelName(model))}
	cfgService.ConvertedAttributes = attrCfg

	deps := setupDeps(attrCfg)

	sensorDeps, err := attrCfg.Validate("path")
	if err != nil {
		return nil, err
	}
	test.That(t, sensorDeps, test.ShouldResemble, attrCfg.Sensors)

	SetCameraValidationMaxTimeoutSecForTesting(1)
	SetDialMaxTimeoutSecForTesting(1)

	svc, err := builtin.NewBuiltIn(ctx, deps, cfgService, logger, bufferSLAMProcessLogs)

	if success {
		if err != nil {
			//return interface{}, err
		}
		test.That(t, svc, test.ShouldNotBeNil)
		return svc, nil
	}

	test.That(t, svc, test.ShouldBeNil)
	// ZACK TODO return interface{}, err
}

func setupDeps(attr *AttrConfig) registry.Dependencies {
	deps := make(registry.Dependencies)
	var projA transform.Projector
	intrinsicsA := &transform.PinholeCameraIntrinsics{ // not the real camera parameters -- fake for test
		Width:  1280,
		Height: 720,
		Fx:     200,
		Fy:     200,
		Ppx:    640,
		Ppy:    360,
	}
	distortionsA := &transform.BrownConrady{RadialK1: 0.001, RadialK2: 0.00004}
	projA = intrinsicsA

	var projRealSense transform.Projector
	intrinsicsRealSense := &transform.PinholeCameraIntrinsics{
		Width:  1280,
		Height: 720,
		Fx:     900.538,
		Fy:     900.818,
		Ppx:    648.934,
		Ppy:    367.736,
	}
	distortionsRealSense := &transform.BrownConrady{
		RadialK1:     0.158701,
		RadialK2:     -0.485405,
		RadialK3:     0.435342,
		TangentialP1: -0.00143327,
		TangentialP2: -0.000705919}
	projRealSense = intrinsicsRealSense

	var projWebcam transform.Projector
	intrinsicsWebcam := &transform.PinholeCameraIntrinsics{
		Width:  640,
		Height: 480,
		Fx:     939.2693584627577,
		Fy:     940.2928257873841,
		Ppx:    320.6075282958033,
		Ppy:    239.14408757087756,
	}
	distortionsWebcam := &transform.BrownConrady{
		RadialK1:     0.046535971648456166,
		RadialK2:     0.8002516496932317,
		RadialK3:     -5.408034254951954,
		TangentialP1: -8.996658362365533e-06,
		TangentialP2: -0.002828504714921335}
	projWebcam = intrinsicsWebcam

	for _, sensor := range attr.Sensors {
		cam := &inject.Camera{}
		switch sensor {
		case "good_lidar":
			cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
				return pointcloud.New(), nil
			}
			cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
				return nil, errors.New("lidar not camera")
			}
			cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
				return nil, transform.NewNoIntrinsicsError("")
			}
			cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
				return camera.Properties{}, nil
			}
			deps[camera.Named(sensor)] = cam
		case "bad_lidar":
			cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
				return nil, errors.New("bad_lidar")
			}
			cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
				return nil, errors.New("lidar not camera")
			}
			cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
				return nil, transform.NewNoIntrinsicsError("")
			}
			deps[camera.Named(sensor)] = cam
		case "good_camera":
			cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
				return gostream.NewEmbeddedVideoStreamFromReader(
					gostream.VideoReaderFunc(func(ctx context.Context) (image.Image, func(), error) {
						return image.NewNRGBA(image.Rect(0, 0, 1024, 1024)), nil, nil
					}),
				), nil
			}
			cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
				return nil, errors.New("camera not lidar")
			}
			cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
				return projA, nil
			}
			cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
				return camera.Properties{IntrinsicParams: intrinsicsA, DistortionParams: distortionsA}, nil
			}
			deps[camera.Named(sensor)] = cam
		case "missing_distortion_parameters_camera":
			cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
				return gostream.NewEmbeddedVideoStreamFromReader(
					gostream.VideoReaderFunc(func(ctx context.Context) (image.Image, func(), error) {
						return image.NewNRGBA(image.Rect(0, 0, 1024, 1024)), nil, nil
					}),
				), nil
			}
			cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
				return nil, errors.New("camera not lidar")
			}
			cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
				return projA, nil
			}
			cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
				return camera.Properties{IntrinsicParams: intrinsicsA, DistortionParams: nil}, nil
			}
			deps[camera.Named(sensor)] = cam
		case "missing_camera_properties":
			cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
				return gostream.NewEmbeddedVideoStreamFromReader(
					gostream.VideoReaderFunc(func(ctx context.Context) (image.Image, func(), error) {
						return image.NewNRGBA(image.Rect(0, 0, 1024, 1024)), nil, nil
					}),
				), nil
			}
			cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
				return nil, errors.New("camera not lidar")
			}
			cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
				return projA, nil
			}
			cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
				return camera.Properties{}, errors.New("somehow couldn't get properties")
			}
			deps[camera.Named(sensor)] = cam
		case "good_color_camera":
			cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
				return nil, errors.New("camera not lidar")
			}
			cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
				return projA, nil
			}
			cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
				return camera.Properties{IntrinsicParams: intrinsicsA, DistortionParams: distortionsA}, nil
			}
			cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
				imgBytes, err := os.ReadFile(artifact.MustPath("rimage/board1.png"))
				if err != nil {
					return nil, err
				}
				lazy := rimage.NewLazyEncodedImage(imgBytes, rdkutils.MimeTypePNG)
				return gostream.NewEmbeddedVideoStreamFromReader(
					gostream.VideoReaderFunc(func(ctx context.Context) (image.Image, func(), error) {
						return lazy, func() {}, nil
					}),
				), nil
			}
			deps[camera.Named(sensor)] = cam
		case "good_depth_camera":
			cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
				return nil, errors.New("camera not lidar")
			}
			cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
				return nil, transform.NewNoIntrinsicsError("")
			}
			cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
				return camera.Properties{}, nil
			}
			cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
				imgBytes, err := os.ReadFile(artifact.MustPath("rimage/board1_gray.png"))
				if err != nil {
					return nil, err
				}
				lazy := rimage.NewLazyEncodedImage(imgBytes, rdkutils.MimeTypePNG)
				return gostream.NewEmbeddedVideoStreamFromReader(
					gostream.VideoReaderFunc(func(ctx context.Context) (image.Image, func(), error) {
						return lazy, func() {}, nil
					}),
				), nil
			}
			deps[camera.Named(sensor)] = cam
		case "bad_camera":
			cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
				return nil, errors.New("bad_camera")
			}
			cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
				return nil, errors.New("camera not lidar")
			}
			cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
				return nil, transform.NewNoIntrinsicsError("")
			}
			deps[camera.Named(sensor)] = cam
		case "bad_camera_intrinsics":
			cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
				return gostream.NewEmbeddedVideoStreamFromReader(
					gostream.VideoReaderFunc(func(ctx context.Context) (image.Image, func(), error) {
						return image.NewNRGBA(image.Rect(0, 0, 1024, 1024)), nil, nil
					}),
				), nil
			}
			cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
				return nil, errors.New("camera not lidar")
			}
			cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
				return &transform.PinholeCameraIntrinsics{}, nil
			}
			cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
				return camera.Properties{
					IntrinsicParams:  &transform.PinholeCameraIntrinsics{},
					DistortionParams: &transform.BrownConrady{},
				}, nil
			}
			deps[camera.Named(sensor)] = cam
		case "orbslam_int_color_camera":
			cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
				return nil, errors.New("camera not lidar")
			}
			cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
				return projRealSense, nil
			}
			cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
				return camera.Properties{IntrinsicParams: intrinsicsRealSense, DistortionParams: distortionsRealSense}, nil
			}
			var index uint64
			cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
				defer func() {
					orbslamIntSynchronizeCamerasChan <- 1
				}()
				// Ensure the StreamFunc functions for orbslam_int_color_camera and orbslam_int_depth_camera run under
				// the lock so that they release images in the same call to getSimultaneousColorAndDepth().
				orbslamIntCameraMutex.Lock()
				select {
				case <-orbslamIntCameraReleaseImagesChan:
					i := atomic.AddUint64(&index, 1) - 1
					if i >= uint64(getNumOrbslamImages(slam.Rgbd)) {
						return nil, errors.New("No more orbslam color images")
					}
					imgBytes, err := os.ReadFile(artifact.MustPath("slam/mock_camera_short/rgb/" + strconv.FormatUint(i, 10) + ".png"))
					if err != nil {
						return nil, err
					}
					lazy := rimage.NewLazyEncodedImage(imgBytes, rdkutils.MimeTypePNG)
					return gostream.NewEmbeddedVideoStreamFromReader(
						gostream.VideoReaderFunc(func(ctx context.Context) (image.Image, func(), error) {
							return lazy, func() {}, nil
						}),
					), nil
				default:
					return nil, errors.Errorf("Color camera not ready to return image %v", index)
				}
			}
			deps[camera.Named(sensor)] = cam
		case "orbslam_int_depth_camera":
			cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
				return nil, errors.New("camera not lidar")
			}
			cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
				return nil, transform.NewNoIntrinsicsError("")
			}
			cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
				return camera.Properties{}, nil
			}
			var index uint64
			cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
				defer func() {
					orbslamIntCameraMutex.Unlock()
				}()
				// Ensure StreamFunc for orbslam_int_color_camera runs first, so that we lock orbslamIntCameraMutex before
				// unlocking it
				<-orbslamIntSynchronizeCamerasChan
				select {
				case <-orbslamIntCameraReleaseImagesChan:
					i := atomic.AddUint64(&index, 1) - 1
					if i >= uint64(getNumOrbslamImages(slam.Rgbd)) {
						return nil, errors.New("No more orbslam depth images")
					}
					imgBytes, err := os.ReadFile(artifact.MustPath("slam/mock_camera_short/depth/" + strconv.FormatUint(i, 10) + ".png"))
					if err != nil {
						return nil, err
					}
					lazy := rimage.NewLazyEncodedImage(imgBytes, rdkutils.MimeTypePNG)
					return gostream.NewEmbeddedVideoStreamFromReader(
						gostream.VideoReaderFunc(func(ctx context.Context) (image.Image, func(), error) {
							return lazy, func() {}, nil
						}),
					), nil
				default:
					return nil, errors.Errorf("Depth camera not ready to return image %v", index)
				}
			}
			deps[camera.Named(sensor)] = cam
		case "orbslam_int_webcam":
			cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
				return nil, errors.New("camera not lidar")
			}
			cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
				return projWebcam, nil
			}
			cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
				return camera.Properties{IntrinsicParams: intrinsicsWebcam, DistortionParams: distortionsWebcam}, nil
			}
			var index uint64
			cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
				select {
				case <-orbslamIntWebcamReleaseImageChan:
					i := atomic.AddUint64(&index, 1) - 1
					if i >= uint64(getNumOrbslamImages(slam.Mono)) {
						return nil, errors.New("No more orbslam webcam images")
					}
					imgBytes, err := os.ReadFile(artifact.MustPath("slam/mock_mono_camera/rgb/" + strconv.FormatUint(i, 10) + ".png"))
					if err != nil {
						return nil, err
					}
					img, _, err := image.Decode(bytes.NewReader(imgBytes))
					if err != nil {
						return nil, err
					}
					var ycbcrImg image.YCbCr
					rimage.ImageToYCbCrForTesting(&ycbcrImg, img)
					return gostream.NewEmbeddedVideoStreamFromReader(
						gostream.VideoReaderFunc(func(ctx context.Context) (image.Image, func(), error) {
							return &ycbcrImg, func() {}, nil
						}),
					), nil
				default:
					return nil, errors.Errorf("Webcam not ready to return image %v", index)
				}
			}
			deps[camera.Named(sensor)] = cam
		case "gibberish":
			return deps
		case "cartographer_int_lidar":
			var index uint64
			cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
				select {
				case <-cartographerIntLidarReleasePointCloudChan:
					i := atomic.AddUint64(&index, 1) - 1
					if i >= numCartographerPointClouds {
						return nil, errors.New("No more cartographer point clouds")
					}
					file, err := os.Open(artifact.MustPath("slam/mock_lidar/" + strconv.FormatUint(i, 10) + ".pcd"))
					if err != nil {
						return nil, err
					}
					pointCloud, err := pointcloud.ReadPCD(file)
					if err != nil {
						return nil, err
					}
					return pointCloud, nil
				default:
					return nil, errors.Errorf("Lidar not ready to return point cloud %v", index)
				}
			}
			cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
				return nil, errors.New("lidar not camera")
			}
			cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
				return nil, transform.NewNoIntrinsicsError("")
			}
			cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
				return camera.Properties{}, nil
			}
			deps[camera.Named(sensor)] = cam
		default:
			continue
		}
	}
	return deps
}
func TestGeneralNew(t *testing.T) {
    t.Run("New slam service blank config", func(t *testing.T) {
        logger := golog.NewTestLogger(t)
        logger.Infoln("test")
        
    })

    // TODO Zack: Ask Kat about the right way to do this
    // See RunTimeConfigValidation in builtin.go (before modification)
//	t.Run("SLAM config check data_rate_ms", func(t *testing.T) {
//		cfg := getValidConfig(name1)
//		model := "cartographer"
//		cfg.DataRateMs = 10
//		_, err = builtin.RuntimeConfigValidation(cfg, model, logger)
//		test.That(t, err, test.ShouldBeError, errors.New("cannot specify data_rate_msec less than 200"))
//	})

}
