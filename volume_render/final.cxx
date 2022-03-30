#include <cmath>
#include <iostream>
#include "vtkSmartPointer.h"
#include "vtkSphereSource.h"
#include "vtkPolyDataMapper.h"
#include "vtkActor.h"
#include "vtkInteractorStyle.h"
#include "vtkObjectFactory.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkProperty.h"
#include "vtkCamera.h"
#include "vtkLight.h"
#include "vtkOpenGLPolyDataMapper.h"
#include "vtkJPEGReader.h"
#include "vtkImageData.h"

#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkPolyDataReader.h>
#include <vtkPoints.h>
#include <vtkUnsignedCharArray.h>
#include <vtkFloatArray.h>
#include <vtkDoubleArray.h>
#include <vtkCellArray.h>
#include <vtkDataSetReader.h>
#include <vtkContourFilter.h>
#include <vtkRectilinearGrid.h>
#include <vtkFloatArray.h>
#include <vtkImageData.h>
#include <vtkPNGWriter.h>


using namespace std;

struct Camera
{
    double          near, far;
    double          angle;
    double          position[3];
    double          focus[3];
    double          up[3];
};


struct TransferFunction
{
    double          min;
    double          max;
    int             numBins;
    unsigned char  *colors;  // size is 3*numBins
    double         *opacities; // size is numBins

    // Take in a value and applies the transfer function.
    // Step #1: figure out which bin "value" lies in.
    // If "min" is 2 and "max" is 4, and there are 10 bins, then
    //   bin 0 = 2->2.2
    //   bin 1 = 2.2->2.4
    //   bin 2 = 2.4->2.6
    //   bin 3 = 2.6->2.8
    //   bin 4 = 2.8->3.0
    //   bin 5 = 3.0->3.2
    //   bin 6 = 3.2->3.4
    //   bin 7 = 3.4->3.6
    //   bin 8 = 3.6->3.8
    //   bin 9 = 3.8->4.0
    // and, for example, a "value" of 3.15 would return the color in bin 5
    // and the opacity at "opacities[5]".
    void ApplyTransferFunction(double value, unsigned char *RGB, double &opacity)
    {
        int bin = GetBin(value);
        if (bin == -9999) {
            RGB[0] = 0;
            RGB[1] = 0;
            RGB[2] = 0;
            opacity = 0;
        } else {
            RGB[0] = colors[3*bin+0];
            RGB[1] = colors[3*bin+1];
            RGB[2] = colors[3*bin+2];
            opacity = opacities[bin];
        }
    }

    int GetBin(double value)
    {
        if ((value < min) || (value > max))
        {
            //cerr << "Out of range: no valid bin" << endl;
            return -9999;
        }
        double bin_num = numBins * ((value - min)/(max - min));
        double ret_val = floor(bin_num);
        //cerr << "Mapped to bin " << ret_val << endl;
        return ret_val;
    }
};

TransferFunction
SetupTransferFunction(void)
{
    int  i;

    TransferFunction rv;
    rv.min = 10;
    rv.max = 15;
    rv.numBins = 256;
    rv.colors = new unsigned char[3*256];
    rv.opacities = new double[256];
    unsigned char charOpacity[256] = {
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 13, 14, 14, 14, 14, 14, 14, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 5, 4, 3, 2, 3, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 17, 17, 17, 17, 17, 17, 16, 16, 15, 14, 13, 12, 11, 9, 8, 7, 6, 5, 5, 4, 3, 3, 3, 4, 5, 6, 7, 8, 9, 11, 12, 14, 16, 18, 20, 22, 24, 27, 29, 32, 35, 38, 41, 44, 47, 50, 52, 55, 58, 60, 62, 64, 66, 67, 68, 69, 70, 70, 70, 69, 68, 67, 66, 64, 62, 60, 58, 55, 52, 50, 47, 44, 41, 38, 35, 32, 29, 27, 24, 22, 20, 20, 23, 28, 33, 38, 45, 51, 59, 67, 76, 85, 95, 105, 116, 127, 138, 149, 160, 170, 180, 189, 198, 205, 212, 217, 221, 223, 224, 224, 222, 219, 214, 208, 201, 193, 184, 174, 164, 153, 142, 131, 120, 109, 99, 89, 79, 70, 62, 54, 47, 40, 35, 30, 25, 21, 17, 14, 12, 10, 8, 6, 5, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        };

    for (i = 0 ; i < 256 ; i++)
        rv.opacities[i] = charOpacity[i]/255.0;
    const int numControlPoints = 8;
    unsigned char controlPointColors[numControlPoints*3] = { 
           71, 71, 219, 0, 0, 91, 0, 255, 255, 0, 127, 0, 
           255, 255, 0, 255, 96, 0, 107, 0, 0, 224, 76, 76 
       };
    double controlPointPositions[numControlPoints] = { 0, 0.143, 0.285, 0.429, 0.571, 0.714, 0.857, 1.0 };
    for (i = 0 ; i < numControlPoints-1 ; i++)
    {
        int start = controlPointPositions[i]*rv.numBins;
        int end   = controlPointPositions[i+1]*rv.numBins+1;
    //cerr << "Working on " << i << "/" << i+1 << ", with range " << start << "/" << end << endl;
        if (end >= rv.numBins)
            end = rv.numBins-1;
        for (int j = start ; j <= end ; j++)
        {
            double proportion = (j/(rv.numBins-1.0)-controlPointPositions[i])/(controlPointPositions[i+1]-controlPointPositions[i]);
            if (proportion < 0 || proportion > 1.)
                continue;
            for (int k = 0 ; k < 3 ; k++)
                rv.colors[3*j+k] = proportion*(controlPointColors[3*(i+1)+k]-controlPointColors[3*i+k])
                                 + controlPointColors[3*i+k];
        }
    }    

    return rv;
}

int GetPointIndex(const int *idx, const int *dims)
{
    return idx[2]*dims[0]*dims[1]+idx[1]*dims[0]+idx[0];
}

double
EvaluateFieldAtLocation(const double *pt, const int *dims, 
                        const float *X, const float *Y, const float *Z, const float *F)
{

    //Establish baseline for coord array
    int coord[3]; coord[0] = -9999; coord[1] = -9999; coord[2] = -9999;

    //Finds location for X value
    for (int i = 0; i < dims[0]; i++) {
        if ((pt[0] < X[i + 1]) && (pt[0] >= X[i])) {
            coord[0] = i;
        }
    }

    //Finds location for Y value
    for (int i = 0; i < dims[1]; i++) {
        if ((pt[1] < Y[i + 1]) && (pt[1] >= Y[i])) {
            coord[1] = i;
        }
    }

    //Finds location for Z value
    for (int i = 0; i < dims[2]; i++) {
        if ((pt[2] < Z[i + 1]) && (pt[2] >= Z[i])) {
            coord[2] = i;
        }
    }

    //If the values didn't change, they were out of bounds
    if ((coord[0] == -9999) || (coord[1] == -9999) || (coord[2] == -9999)) {
        return 0;
    }

    //Verticies of the 8 corners
    int v0[3]; v0[0] = coord[0]; v0[1] = coord[1]; v0[2] = coord[2]; // 000
    int v1[3]; v1[0] = coord[0] + 1; v1[1] = coord[1]; v1[2] = coord[2]; // 100
    int v2[3]; v2[0] = coord[0]; v2[1] = coord[1] + 1; v2[2] = coord[2]; // 010
    int v3[3]; v3[0] = coord[0] + 1; v3[1] = coord[1] + 1; v3[2] = coord[2]; // 110
    int v4[3]; v4[0] = coord[0]; v4[1] = coord[1]; v4[2] = coord[2] + 1; // 001
    int v5[3]; v5[0] = coord[0] + 1; v5[1] = coord[1]; v5[2] = coord[2] + 1; // 101
    int v6[3]; v6[0] = coord[0]; v6[1] = coord[1] + 1; v6[2] = coord[2] + 1; // 011
    int v7[3]; v7[0] = coord[0] + 1; v7[1] = coord[1] + 1; v7[2] = coord[2] + 1; // 111

    //GetPointIndex for each of the corners
    int p_0 = GetPointIndex(v0, dims);
    int p_1 = GetPointIndex(v1, dims);
    int p_2 = GetPointIndex(v2, dims);
    int p_3 = GetPointIndex(v3, dims);
    int p_4 = GetPointIndex(v4, dims);
    int p_5 = GetPointIndex(v5, dims);
    int p_6 = GetPointIndex(v6, dims);
    int p_7 = GetPointIndex(v7, dims);

    //Get the X, Y, and Z values foor each point
    double x_1 = X[coord[0]];
    double x_2 = X[coord[0] + 1];
    double y_1 = Y[coord[1]];
    double y_2 = Y[coord[1] + 1];
    double z_1 = Z[coord[2]];
    double z_2 = Z[coord[2] + 1];

    //Calculate x, y, and z delta
    double xd = (pt[0] - x_1)/(x_2 - x_1);
    double yd = (pt[1] - y_1)/(y_2 - y_1);
    double zd = (pt[2] - z_1)/(z_2 - z_1);

    //Interpolate
    double c00 = F[p_0] * (1.0 - xd) + F[p_1] * xd; //000 //100
    double c01 = F[p_4] * (1.0 - xd) + F[p_5] * xd; //001 //101
    double c10 = F[p_2] * (1.0 - xd) + F[p_3] * xd; //010 //110
    double c11 = F[p_6] * (1.0 - xd) + F[p_7] * xd; //011 //111

    //Interpolate
    double c0 = c00 * (1.0 - yd) + c10 * yd; //000 100 //010 110
    double c1 = c01 * (1.0 - yd) + c11 * yd; //001 101 //011 111

    //Interpolate
    double c_final = c0 * (1.0 - zd) + c1 * zd; //000 100 010 110 // 001 101 011 111

    return c_final;
}

Camera
SetupCamera(void)
{
    Camera rv;
    rv.focus[0] = 0;
    rv.focus[1] = 0;
    rv.focus[2] = 0;
    rv.up[0] = 0;
    rv.up[1] = -1;
    rv.up[2] = 0;
    rv.angle = 30;
    rv.near = 7.5e+7;
    rv.far = 1.4e+8;
    rv.position[0] = -8.25e+7;
    rv.position[1] = -3.45e+7;
    rv.position[2] = 3.35e+7;

    return rv;
}

void
WriteImage(vtkImageData *img, const char *filename)
{
    std::string full_filename = filename;
    full_filename += ".png";
    vtkPNGWriter *writer = vtkPNGWriter::New();
    writer->SetInputData(img);
    writer->SetFileName(full_filename.c_str());
    writer->Write();
    writer->Delete();
}

vtkImageData *
NewImage(int width, int height)
{
    vtkImageData *image = vtkImageData::New();
    image->SetDimensions(width, height, 1);
    image->AllocateScalars(VTK_UNSIGNED_CHAR, 3);

    return image;
}


int main()
{
    Camera c = SetupCamera();
    TransferFunction tf = SetupTransferFunction();

    vtkDataSetReader *reader = vtkDataSetReader::New();
    reader->SetFileName("astro512.vtk");
    reader->Update();

    int dims[3];
    vtkRectilinearGrid *rgrid = (vtkRectilinearGrid *) reader->GetOutput();
    rgrid->GetDimensions(dims);
    int ncells = rgrid->GetNumberOfCells();

    float *X = (float *) rgrid->GetXCoordinates()->GetVoidPointer(0);
    float *Y = (float *) rgrid->GetYCoordinates()->GetVoidPointer(0);
    float *Z = (float *) rgrid->GetZCoordinates()->GetVoidPointer(0);
    float *F = (float *) rgrid->GetPointData()->GetScalars()->GetVoidPointer(0);

    //DIMENSTIONS FOR IMAGE PRODUCED, CHANGE THESE IF YOU WANT TO CHANGE THE DIMENSIONS
    int W = 500;
    int H = 500;

    vtkImageData *image;
    image = NewImage(W, H);
    unsigned char *buffer;

    image->SetOrigin(c.position[0], c.position[1], c.position[2]);
    image->SetSpacing(1.0, 1.0, 1.0);

    // --------------------------- STEP 1 ----------------------------- //
    //Pixels
    double direction[3][W][H];
    for (int k = 0; k < 3; k++) {
        for (int i = 0; i < W; i++) {
            for (int j = 0; j < H; j++) {
                direction[k][i][j] = 0.0;
            }
        }
    }

    double r_angle = (c.angle * (M_PI/180.0));
    double u[3], v[3], dx[3], dy[3], look[3];
    //Initialize
    for (int i = 0; i < 3; i++) {
        u[i] = 0.0;
        v[i] = 0.0;
        dx[i] = 0.0;
        dy[i] = 0.0;
        look[i] = 0.0;
    }

    //look calculation
    look[0] = c.focus[0] - c.position[0];
    look[1] = c.focus[1] - c.position[1];
    look[2] = c.focus[2] - c.position[2];
    //u calculation
    double cprod_u_1 = look[1]*c.up[2] - look[2]*c.up[1];
    double cprod_u_2 = look[2]*c.up[0] - look[0]*c.up[2];
    double cprod_u_3 = look[0]*c.up[1] - look[1]*c.up[0];

    double cprod_u[3] = {cprod_u_1, cprod_u_2, cprod_u_3};
    double normalize_u = sqrt(pow(cprod_u_1, 2.0) + pow(cprod_u_2, 2.0) + pow(cprod_u_3, 2.0));
    
    u[0] = cprod_u[0]/normalize_u;
    u[1] = cprod_u[1]/normalize_u;
    u[2] = cprod_u[2]/normalize_u;
    //v calculation

    double cprod_v_1 = look[1]*u[2] - look[2]*u[1];
    double cprod_v_2 = look[2]*u[0] - look[0]*u[2];
    double cprod_v_3 = look[0]*u[1] - look[1]*u[0];

    double cprod_v[3] = {cprod_v_1, cprod_v_2, cprod_v_3};
    double normalize_v = sqrt(pow(cprod_v_1, 2.0) + pow(cprod_v_2, 2.0) + pow(cprod_v_3, 2.0));

    v[0] = cprod_v[0]/normalize_v;
    v[1] = cprod_v[1]/normalize_v;
    v[2] = cprod_v[2]/normalize_v;
    //dx calculation
    dx[0] = ((2.0 * tan(r_angle/2.0))/ W) * u[0];
    dx[1] = ((2.0 * tan(r_angle/2.0))/ W) * u[1];
    dx[2] = ((2.0 * tan(r_angle/2.0))/ W) * u[2];
    //dy calcukation
    dy[0] = ((2.0 * tan(r_angle/2.0))/ H) * v[0];
    dy[1] = ((2.0 * tan(r_angle/2.0))/ H) * v[1];
    dy[2] = ((2.0 * tan(r_angle/2.0))/ H) * v[2];

    for (int k = 0; k < 3; k++) {

        double normalize_look = sqrt(pow(look[0], 2.0) + pow(look[1], 2.0) + pow(look[2], 2.0));
        for (int i = 0; i < W; i++) {
            for (int j = 0; j < H; j++) {
                direction[k][i][j] = (look[k]/normalize_look) + ((((2.0 * i) + 1.0 - W)/2.0) * dx[k]) + ((((2.0 * j) + 1.0 - H)/2.0) * dy[k]);
            }
        }
    }

    // --------------------------- STEP 2 ----------------------------- //
    //THIS IS THE NUMBER OF SAMPLES TAKEN - 1 <-- If you want 1000 samples to be taken, set to 999.0
    double sampleNumbers = 1023.0; 
    double step_size = (c.far - c.near) / sampleNumbers;

    double retval[3];
    unsigned char RGB[3]; double opacity;
    double new_RGB[3]; double prev_RGB[3];
    double old_opacity; double opacity_ret;

    for (int i = 0; i < W; i++) {
        for (int j = 0; j < H; j++) {
            for (int k = 0; k < sampleNumbers + 1; k++) {
                //Update position <-- current position + (direction * step_size)
                if (k == 0) {
                    retval[0] = c.position[0] + (direction[0][i][j] * c.near);
                    retval[1] = c.position[1] + (direction[1][i][j] * c.near);
                    retval[2] = c.position[2] + (direction[2][i][j] * c.near);
                } else {
                    retval[0] = retval[0] + (direction[0][i][j] * step_size);
                    retval[1] = retval[1] + (direction[1][i][j] * step_size);
                    retval[2] = retval[2] + (direction[2][i][j] * step_size);
                }
                //Evaluate field at each position
                double value = EvaluateFieldAtLocation(retval, dims, X, Y, Z, F);
                //Sort value in bin and apply color
                tf.ApplyTransferFunction(value, RGB, opacity);
                //Adjust opacity A = <-- 1 - (1 - A)^(S_o/S)
                double opacity_adj = 1.0 - pow((1.0 - opacity), 500.0/(sampleNumbers + 1.0));

                //Based on color transparency we find a color for each pixel
                if (k == 0) {
                    new_RGB[0] = 0.0;
                    new_RGB[1] = 0.0;
                    new_RGB[2] = 0.0;
                    opacity_ret = 0.0;
                } else if (k == 1) {
                    new_RGB[0] = old_opacity * prev_RGB[0] + (1.0 - old_opacity) * opacity_adj * (double)RGB[0];
                    new_RGB[1] = old_opacity * prev_RGB[1] + (1.0 - old_opacity) * opacity_adj * (double)RGB[1];
                    new_RGB[2] = old_opacity * prev_RGB[2] + (1.0 - old_opacity) * opacity_adj * (double)RGB[2];
                    opacity_ret = old_opacity + (1.0 - old_opacity) * opacity_adj;
                } else {
                    new_RGB[0] = new_RGB[0] + (1.0 - opacity_ret) * opacity_adj * (double)RGB[0];
                    new_RGB[1] = new_RGB[1] + (1.0 - opacity_ret) * opacity_adj * (double)RGB[1];
                    new_RGB[2] = new_RGB[2] + (1.0 - opacity_ret) * opacity_adj * (double)RGB[2];
                    opacity_ret = opacity_ret + (1.0 - opacity_ret) * opacity_adj;
                }

                //For k == 1 only, finding previous values
                prev_RGB[0] = (double)RGB[0];
                prev_RGB[1] = (double)RGB[1];
                prev_RGB[2] = (double)RGB[2];
                old_opacity = opacity_adj;
            }

            //Apply each color to each pixel
            buffer = (unsigned char *)image->GetScalarPointer(i, j, 0);
            buffer[0] = (unsigned char)(new_RGB[0]);
            buffer[1] = (unsigned char)(new_RGB[1]);
            buffer[2] = (unsigned char)(new_RGB[2]);
        }
    }

    //Writes the image to astro.png
    WriteImage(image, "astro");
}
