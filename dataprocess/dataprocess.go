package dataprocess

import (
    "os"
    "bufio"
)

func WriteImageToFile(image Img, filename string) error {
//nolint:gosec
		f, err := os.Create(filename)
		if err != nil {
			return err
		}
		w := bufio.NewWriter(f)
		if _, err := w.Write(image); err != nil {
			return err
		}
		if err := w.Flush(); err != nil {
			return err
		}
		return f.Close()

}
