#include <iostream>
#include <algorithm>

#include <OpenEXR/ImfRgbaFile.h>
#include <OpenEXR/ImfStringAttribute.h>
#include <OpenEXR/ImfMatrixAttribute.h>
#include <OpenEXR/ImfArray.h>
#include <OpenEXR/ImfInputFile.h>

#include <opencv2/opencv.hpp>
#include <opencv2/videoio/videoio_c.h>

//using namespace Imf;
//using namespace std;
//using namespace IMATH_NAMESPACE;

#define LOG(ERROR) std::cout

void writeSintelDptFile(const char* filename, cv::Mat1f img)
{
	if (filename == nullptr)
	{
		LOG(ERROR) << "Error in writeSintelDptFile: empty filename.";
		return;
	}

	const char* dot = strrchr(filename, '.');
	if (dot == nullptr)
	{
		LOG(ERROR) << "Error in writeSintelDptFile: extension required in filename " << filename << ".";
		return;
	}

	if (strcmp(dot, ".dpt") != 0)
	{
		LOG(ERROR) << "Error in writeSintelDptFile: filename " << filename << " should have extension '.dpt'.";
		return;
	}

	int width = img.cols, height = img.rows, nBands = img.channels();

	if (nBands != 1)
	{
		LOG(ERROR) << "Error in writeSintelDptFile(" << filename << "): image must have 1 channel.";
		return;
	}

	FILE* stream = fopen(filename, "wb");
	if (stream == 0)
	{
		LOG(ERROR) << "Error in writeSintelDptFile: could not open " << filename;
		return;
	}

	// write the header
	fprintf(stream, "PIEH");
	if ((int)fwrite(&width, sizeof(int), 1, stream) != 1
		|| (int)fwrite(&height, sizeof(int), 1, stream) != 1)
	{
		LOG(ERROR) << "Error in writeSintelDptFile(" << filename << "): problem writing header.";
		return;
	}

	// write the rows
	for (int y = 0; y < height; y++)
	{
		if ((int)fwrite(img.row(y).data, sizeof(float), width, stream) != width)
		{
			LOG(ERROR) << "Error in writeSintelDptFile(" << filename << "): problem writing data.";
			return;
		}
	}

	fclose(stream);
}

int main(int argc, char* argv[])
{

    if (argc == 1){
        std::string("Provide directory with EXR files");
        exit(-1);
    }

    std::string input_dir = argv[1];
    if (!input_dir.empty() && input_dir.back() != '/')
        input_dir += '/';

    std::vector<cv::String> fn;
    cv::glob(input_dir + "*.exr", fn, false);
    for (int i=0; i<fn.size(); i++) {
        std::string inputFileName, outputFileName;
        inputFileName = fn.at(i);
        std::cout << inputFileName << std::endl;
        std::string base_filename = inputFileName.substr(inputFileName.find_last_of("/\\") + 1);
        outputFileName = input_dir + base_filename.replace(base_filename.find("exr"), 3, "dpt").c_str();
//        std::stringstream ss;
//        ss << std::setfill('0') << std::setw(5) << i << ".dpt";
        Imf::InputFile file_rgb_float(inputFileName.c_str());

        Imf::Array2D<float> rPixels;
        Imf::Array2D<float> gPixels;
        Imf::Array2D<float> bPixels;

        Imath::Box2i dw = file_rgb_float.header().dataWindow();

        int width = dw.max.x - dw.min.x + 1;
        int height = dw.max.y - dw.min.y + 1;

        rPixels.resizeErase(height, width);
        gPixels.resizeErase(height, width);
        bPixels.resizeErase(height, width);


        Imf::FrameBuffer frameBuffer;

        frameBuffer.insert("R", // name
                           Imf::Slice(Imf::FLOAT, // type
                                      (char *) (&rPixels[0][0] - // base
                                                dw.min.x -
                                                dw.min.y * width),
                                      sizeof(rPixels[0][0]) * 1, // xStride
                                      sizeof(rPixels[0][0]) * width,// yStride
                                      1, 1, // x/y sampling
                                      0.0)); // fillValue
        frameBuffer.insert("G", // name
                           Imf::Slice(Imf::FLOAT, // type
                                      (char *) (&gPixels[0][0] - // base
                                                dw.min.x -
                                                dw.min.y * width),
                                      sizeof(gPixels[0][0]) * 1, // xStride
                                      sizeof(gPixels[0][0]) * width,// yStride
                                      1, 1, // x/y sampling
                                      0.0)); // fillValue

        frameBuffer.insert("B", // name
                           Imf::Slice(Imf::FLOAT, // type
                                      (char *) (&bPixels[0][0] - // base
                                                dw.min.x -
                                                dw.min.y * width),
                                      sizeof(bPixels[0][0]) * 1, // xStride
                                      sizeof(bPixels[0][0]) * width,// yStride
                                      1, 1, // x/y sampling
                                      FLT_MAX)); // fillValue

        file_rgb_float.setFrameBuffer(frameBuffer);
        file_rgb_float.readPixels(dw.min.y, dw.max.y);

        // convert to opencv mat format
        cv::Mat r_mat = cv::Mat::zeros(height, width, CV_32FC1);
        //memcpy(r_mat.data, R.base, 3*1.0*sizeof(float)*height * width);
        memcpy(r_mat.data, bPixels[0], 1 * 1.0 * sizeof(float) * height * width);

        // set the invalid depth value to 0
        cv::threshold(r_mat, r_mat, 1000000, 0, cv::THRESH_TOZERO_INV);

        // out to DPT file

        writeSintelDptFile(outputFileName.c_str(), r_mat);


//        cv::Mat r_mat_float;
        //r_mat.convertTo(r_mat_float, CV_32FC4);
        //cv::Mat of = comput_of(r_mat_float);

        //cv::Mat src_img;
        //src_img = cv::imread("d:/0001.png");
        //cv::Mat src_img_warpped;
        //warp_image_forward(src_img, of, src_img_warpped);
        //cv::imwrite("d:/temp.png", src_img_warpped);
    }
	return 0;
}
