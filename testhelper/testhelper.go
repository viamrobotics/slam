// Package testhelper provides helper functions for testing implementations of slam libraries
package testhelper

import (
	"errors"
	"io/ioutil"
	"os"
	"testing"

	"go.viam.com/rdk/services/slam"
	"go.viam.com/test"
)

// CreateTempFolderArchitecture creates a new random temporary
// directory with the config, data, and map subdirectories needed
// to run the SLAM libraries.
func CreateTempFolderArchitecture() (string, error) {
	name, err := os.MkdirTemp("", "*")
	if err != nil {
		return "nil", err
	}

	if err := os.Mkdir(name+"/config", os.ModePerm); err != nil {
		return "", err
	}
	if err := os.Mkdir(name+"/data", os.ModePerm); err != nil {
		return "", err
	}
	if err := os.Mkdir(name+"/map", os.ModePerm); err != nil {
		return "", err
	}

	return name, nil
}

// ResetFolder removes all content in path and creates a new directory
// in its place.
func ResetFolder(path string) error {
	err := os.RemoveAll(path)
	if err != nil {
		return err
	}
	err = os.Mkdir(path, os.ModePerm)
	return err
}

func CheckDeleteProcessedData(t *testing.T, subAlgo slam.SubAlgo, dir string, prev int, deleteProcessedData, useLiveData bool) int {
	var numFiles int

	switch subAlgo {
	case slam.Mono:
		numFilesRGB, err := checkDataDirForExpectedFiles(t, dir+"/data/rgb", prev, deleteProcessedData, useLiveData)
		test.That(t, err, test.ShouldBeNil)

		numFiles = numFilesRGB
	case slam.Rgbd:
		numFilesRGB, err := checkDataDirForExpectedFiles(t, dir+"/data/rgb", prev, deleteProcessedData, useLiveData)
		test.That(t, err, test.ShouldBeNil)

		numFilesDepth, err := checkDataDirForExpectedFiles(t, dir+"/data/depth", prev, deleteProcessedData, useLiveData)
		test.That(t, err, test.ShouldBeNil)

		test.That(t, numFilesRGB, test.ShouldEqual, numFilesDepth)
		numFiles = numFilesRGB
	case slam.Dim2d:
		numFiles2D, err := checkDataDirForExpectedFiles(t, dir+"/data", prev, deleteProcessedData, useLiveData)
		test.That(t, err, test.ShouldBeNil)
		numFiles = numFiles2D
	default:
	}
	return numFiles
}

// checkDataDirForExpectedFiles compares the number of files found in a specified data
// directory with the previous number found and uses the online state and
// delete_processed_data value to evaluate this comparison.
func checkDataDirForExpectedFiles(t *testing.T, dir string, prev int, delete_processed_data, useLiveData bool) (int, error) {

	files, err := ioutil.ReadDir(dir)
	test.That(t, err, test.ShouldBeNil)

	if prev == 0 {
		return len(files), nil
	}
	if delete_processed_data && useLiveData {
		test.That(t, prev, test.ShouldBeLessThanOrEqualTo, dataBufferSize+1)
	}
	if !delete_processed_data && useLiveData {
		test.That(t, prev, test.ShouldBeLessThan, len(files))
	}
	if delete_processed_data && !useLiveData {
		return 0, errors.New("the delete_processed_data value cannot be true when running SLAM in offline mode")
	}
	if !delete_processed_data && !useLiveData {
		test.That(t, prev, test.ShouldEqual, len(files))
	}
	return len(files), nil
}
