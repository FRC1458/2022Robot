package frc.robot.pipelines;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.HashMap;

import edu.wpi.first.vision.VisionPipeline;

import org.opencv.core.*;
import org.opencv.core.Core.*;
import org.opencv.features2d.SimpleBlobDetector;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.*;
import org.opencv.objdetect.*;

/**
* RBDPipeline class.
*
* <p>An OpenCV pipeline generated by GRIP.
*
* @author GRIP
*/
public class RBDPipeline implements VisionPipeline {

	//Outputs
	private Mat hsvThreshold0Output = new Mat();
	private Mat cvBitwiseNotOutput = new Mat();
	private Mat maskOutput = new Mat();
	private Mat hsvThreshold1Output = new Mat();
	private MatOfKeyPoint findBlobsOutput = new MatOfKeyPoint();

	static {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}

	/**
	 * This is the primary method that runs the entire pipeline and updates the outputs.
	 */
	@Override	public void process(Mat source0) {
		// Step HSV_Threshold0:
		Mat hsvThreshold0Input = source0;
		double[] hsvThreshold0Hue = {101.97841726618705, 167.87878787878788};
		double[] hsvThreshold0Saturation = {0.0, 255.0};
		double[] hsvThreshold0Value = {0.0, 255.0};
		hsvThreshold(hsvThreshold0Input, hsvThreshold0Hue, hsvThreshold0Saturation, hsvThreshold0Value, hsvThreshold0Output);

		// Step CV_bitwise_not0:
		Mat cvBitwiseNotSrc1 = hsvThreshold0Output;
		cvBitwiseNot(cvBitwiseNotSrc1, cvBitwiseNotOutput);

		// Step Mask0:
		Mat maskInput = source0;
		Mat maskMask = cvBitwiseNotOutput;
		mask(maskInput, maskMask, maskOutput);

		// Step HSV_Threshold1:
		Mat hsvThreshold1Input = maskOutput;
		double[] hsvThreshold1Hue = {0.0, 180.0};
		double[] hsvThreshold1Saturation = {107.22642152858074, 255.0};
		double[] hsvThreshold1Value = {48.156474820143885, 255.0};
		hsvThreshold(hsvThreshold1Input, hsvThreshold1Hue, hsvThreshold1Saturation, hsvThreshold1Value, hsvThreshold1Output);

		// Step Find_Blobs0:
		Mat findBlobsInput = hsvThreshold1Output;
		double findBlobsMinArea = 100.0;
		double[] findBlobsCircularity = {0.5215827338129496, 1.0};
		boolean findBlobsDarkBlobs = false;
		findBlobs(findBlobsInput, findBlobsMinArea, findBlobsCircularity, findBlobsDarkBlobs, findBlobsOutput);

	}

	/**
	 * This method is a generated getter for the output of a HSV_Threshold.
	 * @return Mat output from HSV_Threshold.
	 */
	public Mat hsvThreshold0Output() {
		return hsvThreshold0Output;
	}

	/**
	 * This method is a generated getter for the output of a CV_bitwise_not.
	 * @return Mat output from CV_bitwise_not.
	 */
	public Mat cvBitwiseNotOutput() {
		return cvBitwiseNotOutput;
	}

	/**
	 * This method is a generated getter for the output of a Mask.
	 * @return Mat output from Mask.
	 */
	public Mat maskOutput() {
		return maskOutput;
	}

	/**
	 * This method is a generated getter for the output of a HSV_Threshold.
	 * @return Mat output from HSV_Threshold.
	 */
	public Mat hsvThreshold1Output() {
		return hsvThreshold1Output;
	}

	/**
	 * This method is a generated getter for the output of a Find_Blobs.
	 * @return MatOfKeyPoint output from Find_Blobs.
	 */
	public MatOfKeyPoint findBlobsOutput() {
		return findBlobsOutput;
	}


	/**
	 * Computes the per element inverse of an image.
	 * @param src the image to invert.
	 * @param dst the inversion of the input image.
	 */
	private void cvBitwiseNot(Mat src, Mat dst) {
		Core.bitwise_not(src, dst);
	}

	/**
	 * Filter out an area of an image using a binary mask.
	 * @param input The image on which the mask filters.
	 * @param mask The binary image that is used to filter.
	 * @param output The image in which to store the output.
	 */
	private void mask(Mat input, Mat mask, Mat output) {
		mask.convertTo(mask, CvType.CV_8UC1);
		Core.bitwise_xor(output, output, output);
		input.copyTo(output, mask);
	}

	/**
	 * Segment an image based on hue, saturation, and value ranges.
	 *
	 * @param input The image on which to perform the HSL threshold.
	 * @param hue The min and max hue
	 * @param sat The min and max saturation
	 * @param val The min and max value
	 * @param output The image in which to store the output.
	 */
	private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val,
	    Mat out) {
		Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HSV);
		Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
			new Scalar(hue[1], sat[1], val[1]), out);
	}

	/**
	 * Detects groups of pixels in an image.
	 * @param input The image on which to perform the find blobs.
	 * @param minArea The minimum size of a blob that will be found
	 * @param circularity The minimum and maximum circularity of blobs that will be found
	 * @param darkBlobs The boolean that determines if light or dark blobs are found.
	 * @param blobList The output where the MatOfKeyPoint is stored.
	 */
	private void findBlobs(Mat input, double minArea, double[] circularity,
		Boolean darkBlobs, MatOfKeyPoint blobList) {
		SimpleBlobDetector blobDet = SimpleBlobDetector.create();
		try {
			File tempFile = File.createTempFile("config", ".xml");

			StringBuilder config = new StringBuilder();

			config.append("<?xml version=\"1.0\"?>\n");
			config.append("<opencv_storage>\n");
			config.append("<thresholdStep>10.</thresholdStep>\n");
			config.append("<minThreshold>50.</minThreshold>\n");
			config.append("<maxThreshold>220.</maxThreshold>\n");
			config.append("<minRepeatability>2</minRepeatability>\n");
			config.append("<minDistBetweenBlobs>10.</minDistBetweenBlobs>\n");
			config.append("<filterByColor>1</filterByColor>\n");
			config.append("<blobColor>");
			config.append((darkBlobs ? 0 : 255));
			config.append("</blobColor>\n");
			config.append("<filterByArea>1</filterByArea>\n");
			config.append("<minArea>");
			config.append(minArea);
			config.append("</minArea>\n");
			config.append("<maxArea>");
			config.append(Integer.MAX_VALUE);
			config.append("</maxArea>\n");
			config.append("<filterByCircularity>1</filterByCircularity>\n");
			config.append("<minCircularity>");
			config.append(circularity[0]);
			config.append("</minCircularity>\n");
			config.append("<maxCircularity>");
			config.append(circularity[1]);
			config.append("</maxCircularity>\n");
			config.append("<filterByInertia>1</filterByInertia>\n");
			config.append("<minInertiaRatio>0.1</minInertiaRatio>\n");
			config.append("<maxInertiaRatio>" + Integer.MAX_VALUE + "</maxInertiaRatio>\n");
			config.append("<filterByConvexity>1</filterByConvexity>\n");
			config.append("<minConvexity>0.95</minConvexity>\n");
			config.append("<maxConvexity>" + Integer.MAX_VALUE + "</maxConvexity>\n");
			config.append("</opencv_storage>\n");
			FileWriter writer;
			writer = new FileWriter(tempFile, false);
			writer.write(config.toString());
			writer.close();
			blobDet.read(tempFile.getPath());
		} catch (IOException e) {
			e.printStackTrace();
		}

		blobDet.detect(input, blobList);
	}




}
