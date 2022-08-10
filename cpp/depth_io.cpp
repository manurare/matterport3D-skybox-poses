#include "depth_io.hpp"
#include <iostream>

// Reads a .dpt file (Sintel format) into an OpenCV matrix.
cv::Mat1f readSintelDptFile(const char* filename)
{
    if (filename == nullptr)
    {
        std::cout << "Error in readSintelDptFile: empty filename.";
        return cv::Mat1f();
    }

    FILE* stream = fopen(filename, "rb");
    if (stream == 0)
    {
        std::cout << "Error in readSintelDptFile: could not open " << filename << ".";
        return cv::Mat1f();
    }

    int width, height;
    float tag;

    if ((int)fread(&tag, sizeof(float), 1, stream) != 1
        || (int)fread(&width, sizeof(int), 1, stream) != 1
        || (int)fread(&height, sizeof(int), 1, stream) != 1)
    {
        std::cout << "Error in readSintelDptFile: problem reading file " << filename << ".";
        return cv::Mat1f();
    }

    if (tag != 202021.25) // simple test for correct endian-ness
    {
        std::cout << "Error in readSintelDptFile(" << filename << "): wrong tag (possibly due to big-endian machine?).";
        return cv::Mat1f();
    }

    // another sanity check to see that integers were read correctly (99999 should do the trick...)
    if (width < 1 || width > 99999)
    {
        std::cout << "Error in readSintelDptFile(" << filename << "): illegal width " << width << ".";
        return cv::Mat1f();
    }

    if (height < 1 || height > 99999)
    {
        std::cout << "Error in readSintelDptFile(" << filename << "): illegal height" << height << ".";
        return cv::Mat1f();
    }

    cv::Mat1f img(height, width);

    const size_t pixels = (size_t)width * (size_t)height;
    if (fread(img.data, sizeof(float), pixels, stream) != pixels)
    {
        std::cout << "Error in readSintelDptFile(" << filename << "): file is too short.";
        return cv::Mat1f();
    }

    if (fgetc(stream) != EOF)
    {
        std::cout << "Error in readSintelDptFile(" << filename << "): file is too long.";
        return cv::Mat1f();
    }

    fclose(stream);
    return img;
}

// Reads a .dpt file (Sintel format) into an OpenCV matrix.
cv::Mat1f readSintelDptFile(std::string filename)
{
    return readSintelDptFile(filename.c_str());
}


// Writes a depth map to a .dpt file (Sintel format).
void writeSintelDptFile(const char* filename, cv::Mat1f img)
{
    if (filename == nullptr)
    {
        std::cout << "Error in writeSintelDptFile: empty filename.";
        return;
    }

    const char* dot = strrchr(filename, '.');
    if (dot == nullptr)
    {
        std::cout << "Error in writeSintelDptFile: extension required in filename " << filename << ".";
        return;
    }

    if (strcmp(dot, ".dpt") != 0)
    {
        std::cout << "Error in writeSintelDptFile: filename " << filename << " should have extension '.dpt'.";
        return;
    }

    int width = img.cols, height = img.rows, nBands = img.channels();

    if (nBands != 1)
    {
        std::cout << "Error in writeSintelDptFile(" << filename << "): image must have 1 channel.";
        return;
    }

    FILE* stream = fopen(filename, "wb");
    if (stream == 0)
    {
        std::cout << "Error in writeSintelDptFile: could not open " << filename;
        return;
    }

    // write the header
    fprintf(stream, "PIEH");
    if ((int)fwrite(&width, sizeof(int), 1, stream) != 1
        || (int)fwrite(&height, sizeof(int), 1, stream) != 1)
    {
        std::cout << "Error in writeSintelDptFile(" << filename << "): problem writing header.";
        return;
    }

    // write the rows
    for (int y = 0; y < height; y++)
    {
        if ((int)fwrite(img.row(y).data, sizeof(float), width, stream) != width)
        {
            std::cout << "Error in writeSintelDptFile(" << filename << "): problem writing data.";
            return;
        }
    }

    fclose(stream);
}

// Writes a depth map to a .dpt file (Sintel format).
void writeSintelDptFile(std::string filename, cv::Mat1f img)
{
    writeSintelDptFile(filename.c_str(), img);
}
