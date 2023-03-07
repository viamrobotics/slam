// Package dataprocess manages code related to the data-saving process
package dataprocess

import (
	"bufio"
	"bytes"
	"context"
	"os"

	pc "go.viam.com/rdk/pointcloud"
)

// WritePCDToFile encodes the pointcloud and then saves it to the passed filename.
func WritePCDToFile(pointcloud pc.PointCloud, filename string) error {
	buf := new(bytes.Buffer)
	err := pc.ToPCD(pointcloud, buf, 1)
	if err != nil {
		return err
	}
	return WriteBytesToFile(buf.Bytes(), filename)
}

// WriteBytesToFile writes the passed bytes to the passed filename.
func WriteBytesToFile(bytes []byte, filename string) error {
	//nolint:gosec
	f, err := os.Create(filename)
	if err != nil {
		return err
	}
	w := bufio.NewWriter(f)
	if _, err := w.Write(bytes); err != nil {
		return err
	}
	if err := w.Flush(); err != nil {
		return err
	}
	return f.Close()
}
