#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include "Eigen.h"
#include "VirtualSensor.h"

using namespace std;

struct Vertex
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	// position stored as 4 floats (4th component is supposed to be 1.0)
	Vector4f position;
	// color stored as 4 unsigned char
	Vector4uc color;
};

float euclideanDistance(Vertex vertex0, Vertex vertex1){
	return (float)sqrt(pow(vertex0.position[0]-vertex1.position[0],2) + 
		pow(vertex0.position[1]-vertex1.position[1],2) + pow(vertex0.position[2]-vertex1.position[2],2));;
}

bool ValidateVertices(Vertex* vertices, Vector3d face, float edgeThreshold){
	struct Vertex vertex0 = vertices[(int)face[0]];
	struct Vertex vertex1 = vertices[(int)face[1]];
	struct Vertex vertex2 = vertices[(int)face[2]];
	if(vertex0.position.x()==MINF || vertex1.position.x()==MINF || vertex2.position.x()==MINF){
		return false;
	}
	if(euclideanDistance(vertex0, vertex1) > edgeThreshold){
		return false;
	}
	if(euclideanDistance(vertex1, vertex2) > edgeThreshold){
		return false;
	}
	if(euclideanDistance(vertex0, vertex2) > edgeThreshold){
		return false;
	}
	return true;
}

bool WriteMesh(Vertex* vertices, unsigned int width, unsigned int height, const std::string& filename)
{
	float edgeThreshold = 0.01f; // 1cm

	// TODO 2: use the OFF file format to save the vertices grid (http://www.geomview.org/docs/html/OFF.html)
	// - have a look at the "off_sample.off" file to see how to store the vertices and triangles
	// - for debugging we recommend to first only write out the vertices (set the number of faces to zero)
	// - for simplicity write every vertex to file, even if it is not valid (position.x() == MINF) (note that all vertices in the off file have to be valid, thus, if a point is not valid write out a dummy point like (0,0,0))
	// - use a simple triangulation exploiting the grid structure (neighboring vertices build a triangle, two triangles per grid cell)
	// - you can use an arbitrary triangulation of the cells, but make sure that the triangles are consistently oriented
	// - only write triangles with valid vertices and an edge length smaller then edgeThreshold

	// TODO: Get number of vertices
	unsigned int nVertices = width*height;

	// TODO: calculate and store faces
	vector<Vector3d> faces;
	for(int j=0;j<height*(width-1);j=j+width){
		for(int i=0;i<width-1;i++){
			Vector3d face1 = Vector3d(j+i, j+width+i, j+1+i);
			if(ValidateVertices(vertices, face1, edgeThreshold)){
				faces.push_back(face1);
			}
			Vector3d face2 = Vector3d(j+width+i, j+width+i+1, j+1+i);
			if(ValidateVertices(vertices, face2, edgeThreshold)){
				faces.push_back(face2);
			}
		}
	}

	// TODO: Get number of faces
	unsigned nFaces = faces.size();
	std::cout<<nFaces<<std::endl;

	// Write off file
	std::ofstream outFile(filename);
	if (!outFile.is_open()) return false;

	// write header
	outFile << "COFF" << std::endl;
	outFile << nVertices << " " << nFaces << " 0" << std::endl;

	// TODO: save vertices
	for(int i=0; i<nVertices; i++){
		Vector4f position = vertices[i].position;
		Vector4uc color = vertices[i].color;
		if(position.x() == MINF){
			outFile<<"0 0 0 0 0 0 0"<<std::endl;
			continue;
		}
		outFile<<position.x()<<" "<<position.y()<<" "<<position.z()<<" ";
		outFile<<(int)color.x()<<" "<<(int)color.y()<<" "<<(int)color.z()<<" "<<(int)color.w()<<std::endl;
	}

	// TODO: save faces
	for(int i=0; i<nFaces; i++){
		Vector3d face = faces.at(i);
		outFile<<"3 "<<face.x()<<" "<<face.y()<<" "<<face.z()<<std::endl;
	}

	// close file
	outFile.close();

	return true;
}

int main()
{
	std::string filenameIn = "./data/rgbd_dataset_freiburg1_xyz/";
	std::string filenameBaseOut = "mesh_";

	// load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	if (!sensor.Init(filenameIn))
	{
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
	}

	// convert video to meshes
	while (sensor.ProcessNextFrame())
	{
		// get ptr to the current depth frame
		// depth is stored in row major (get dimensions via sensor.GetDepthImageWidth() / GetDepthImageHeight())
		float* depthMap = sensor.GetDepth();
		// get ptr to the current color frame
		// color is stored as RGBX in row major (4 byte values per pixel, get dimensions via sensor.GetColorImageWidth() / GetColorImageHeight())
		BYTE* colorMap = sensor.GetColorRGBX();

		// get depth intrinsics
		Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
		float fovX = depthIntrinsics(0, 0);
		float fovY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);

		// compute inverse depth extrinsics
		Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();

		Matrix4f trajectory = sensor.GetTrajectory();
		Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();

		// TODO 1: back-projection
		// write result to the vertices array below, keep pixel ordering!
		// if the depth value at idx is invalid (MINF) write the following values to the vertices array
		// vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
		// vertices[idx].color = Vector4uc(0,0,0,0);
		// otherwise apply back-projection and transform the vertex to world space, use the corresponding color from the colormap
		Vertex* vertices = new Vertex[sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight()];
		int numPixels = sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight();
		for(int i=0; i<numPixels; i++){
			float z = depthMap[i];
			if(z==MINF){
				vertices[i].position = Vector4f(MINF, MINF, MINF, MINF);
				vertices[i].color = Vector4uc(0,0,0,0);
				continue;
			}
			float pixelX = i/sensor.GetDepthImageWidth();
			float pixelY = i%sensor.GetDepthImageWidth();
			float x = ((pixelX - cX) * z) / fovX;
			float y = ((pixelY - cY) * z) / fovY;
			vertices[i].position = Vector4f(x, y, z, 1.0f);
			vertices[i].position = trajectoryInv * depthExtrinsicsInv * vertices[i].position;
			int ci = i*4;
			vertices[i].color = Vector4uc(colorMap[ci], colorMap[ci+1], colorMap[ci+2], colorMap[ci+3]);
		}

		// write mesh file
		std::stringstream ss;
		ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
		if (!WriteMesh(vertices, sensor.GetDepthImageWidth(), sensor.GetDepthImageHeight(), ss.str()))
		{
			std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
			return -1;
		}

		// free mem
		delete[] vertices;
	}

	return 0;
}
