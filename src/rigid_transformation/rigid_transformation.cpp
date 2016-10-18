#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>
#include "rigid_transformation.h"

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/Exporter.hpp>      // C++ exporter interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing fla



template<typename Type>
void copy_from_mesh(
	const aiMesh* mesh, 
	std::vector<Eigen::Matrix<Type, 3, 1>>& vertices, 
	std::vector<Eigen::Matrix<Type, 3, 1>>& normals)
{
	vertices.clear();
	normals.clear();
	for (size_t i = 0; i < mesh->mNumVertices; ++i)
	{
		const aiVector3D pos = mesh->mVertices[i];
		vertices.push_back(Eigen::Matrix<Type, 3, 1>(pos.x, pos.y, pos.z));

		const aiVector3D normal = mesh->mNormals[i];
		normals.push_back(Eigen::Matrix<Type, 3, 1>(normal.x, normal.y, normal.z));
	}
}


template<typename Type>
void copy_to_mesh(
	const std::vector<Eigen::Matrix<Type, 3, 1>>& vertices,
	const std::vector<Eigen::Matrix<Type, 3, 1>>& normals,
	aiMesh*& mesh)
{
	for (size_t i = 0; i < mesh->mNumVertices; ++i)
	{
		aiVector3D& vertex = mesh->mVertices[i];
		memcpy(&vertex, vertices[i].data(), sizeof(Type) * 3);

		aiVector3D& normal = mesh->mNormals[i];
		memcpy(&normal, normals[i].data(), sizeof(Type) * 3);
	}
}


template<typename Type>
void apply_random_rotation(
	const std::vector<Eigen::Matrix<Type, 3, 1>>& in_vertices,
	const std::vector<Eigen::Matrix<Type, 3, 1>>& in_normals,
	std::vector<Eigen::Matrix<Type, 3, 1>>& rot_vertices,
	std::vector<Eigen::Matrix<Type, 3, 1>>& rot_normals)
{
	rot_vertices.clear();
	rot_normals.clear();

	assert(in_vertices.size() == in_normals.size());

	rot_vertices.resize(in_vertices.size());
	rot_normals.resize(in_normals.size());

	Eigen::Transform<Type, 3, Eigen::Affine> transform;
	transform.setIdentity();
	transform.rotate(Eigen::AngleAxis<Type>(DegToRad(Type(90)), Eigen::Matrix<Type, 3, 1>::Random()));

	for (std::size_t i = 0; i < in_vertices.size(); ++i)
	{
		const Eigen::Matrix<Type, 4, 1>& v = in_vertices[i].homogeneous();
		const Eigen::Matrix<Type, 4, 1>& n = in_normals[i].homogeneous();

		Eigen::Matrix<Type, 4, 1> rv = transform.matrix() * v;
		rv /= rv.w();

		Eigen::Matrix<Type, 4, 1> rn = transform.matrix() * n;
		rn /= rn.w();

		rot_vertices.at(i) = (rv.head<3>());
		rot_normals.at(i) = (rn.head<3>()).normalized();
	}

}


template<typename Type>
void apply_transform(
	const std::vector<Eigen::Matrix<Type, 3, 1>>& in_vertices,
	const std::vector<Eigen::Matrix<Type, 3, 1>>& in_normals,
	const Eigen::Matrix<Type, 4, 4>& transform,
	std::vector<Eigen::Matrix<Type, 3, 1>>& out_vertices,
	std::vector<Eigen::Matrix<Type, 3, 1>>& out_normals)
{
	out_vertices.clear();
	out_normals.clear();

	assert(in_vertices.size() == in_normals.size());

	out_vertices.resize(in_vertices.size());
	out_normals.resize(in_normals.size());

	for (std::size_t i = 0; i < in_vertices.size(); ++i)
	{
		const Eigen::Matrix<Type, 4, 1>& v = in_vertices[i].homogeneous();
		const Eigen::Matrix<Type, 4, 1>& n = in_normals[i].homogeneous();

		Eigen::Matrix<Type, 4, 1> tv = transform.matrix() * v;
		tv /= tv.w();

		Eigen::Matrix<Type, 4, 1> tn = transform.matrix() * n;
		tn /= tn.w();

		out_vertices.at(i) = (tv.head<3>());
		out_normals.at(i) = (tn.head<3>()).normalized();
	}

}



//#define ASSIMP_DOUBLE_PRECISION	// if defined, use double instead float
typedef ai_real Decimal;

int main(int argc, char* argv[])
{
	std::cout
		<< std::fixed << std::endl
		<< "Usage            : ./<app.exe> <input_model> <output_format> " << std::endl
		<< "Default          : ./rigid_transformation.exe ../../data/teddy.obj obj" << std::endl
		<< std::endl;
	
	std::string input_filename = "../../data/teddy.obj";
	std::string output_format = input_filename.substr(input_filename.size() - 3, 3);

	if (argc > 1)
		input_filename = argv[1];
	if (argc > 2)
		output_format = argv[2];
	
	
	//
	// Import file
	// 
	Assimp::Importer importer;
	const aiScene *scene = importer.ReadFile(input_filename, aiProcessPreset_TargetRealtime_Fast);//aiProcessPreset_TargetRealtime_Fast has the configs you'll need
	if (scene == nullptr)
	{
		std::cout << "Error: Could not read file: " << input_filename << std::endl;
		return EXIT_FAILURE;
	}


	//
	// Output info
	// 
	aiMesh *mesh = scene->mMeshes[0]; //assuming you only want the first mesh
	std::cout
		<< "Input File       : " << input_filename << std::endl
		<< "Vertices         : " << mesh->mNumVertices << std::endl
		<< "Faces            : " << mesh->mNumFaces << std::endl;

	std::vector<Eigen::Matrix<Decimal, 3, 1>> in_vertices;
	std::vector<Eigen::Matrix<Decimal, 3, 1>> in_normals;
	copy_from_mesh(mesh, in_vertices, in_normals);
	

	std::vector<Eigen::Matrix<Decimal, 3, 1>> rot_vertices;
	std::vector<Eigen::Matrix<Decimal, 3, 1>> rot_normals;
	apply_random_rotation(in_vertices, in_normals, rot_vertices, rot_normals);
	
	copy_to_mesh(rot_vertices, rot_normals, mesh);


	//
	// Composing file name
	// 
	std::stringstream ss;
	ss << input_filename.substr(0, input_filename.size() - 4)
		<< "_random_transformed." << output_format;
	std::string random_transf_filename = ss.str();

	//
	// Exporting random transformed vertices
	// 
	Assimp::Exporter exporter;
	aiReturn ret = exporter.Export(scene, output_format, random_transf_filename, scene->mFlags);
	if (ret == aiReturn_SUCCESS)
		std::cout << "Transformed file : " << random_transf_filename << std::endl;
	else
		std::cout << "Transformed file : <ERROR> file not saved - " << random_transf_filename << std::endl;


	//
	// Computing rigid transformation
	// 
	Eigen::Matrix<Decimal, 4, 4> transform;
	compute_rigid_transformation(in_vertices, rot_vertices, transform);

	std::cout
		<< std::endl
		<< "Transform Matrix : " << std::endl
		<< transform << std::endl 
		<< std::endl;


	std::vector<Eigen::Matrix<Decimal, 3, 1>> out_vertices;
	std::vector<Eigen::Matrix<Decimal, 3, 1>> out_normals;
	apply_transform(in_vertices, in_normals, transform, out_vertices, out_normals);



	//
	// Composing file name
	// 
	std::stringstream result_ss;
	result_ss << input_filename.substr(0, input_filename.size() - 4)
		<< "_result_transformed." << output_format;
	std::string result_transf_filename = result_ss.str();

	//
	// Exporting random transformed vertices
	// 
	ret = exporter.Export(scene, output_format, result_transf_filename, scene->mFlags);
	if (ret == aiReturn_SUCCESS)
		std::cout << "Result File      : " << result_transf_filename << std::endl;
	else
		std::cout << "Result File      : <ERROR> file not saved - " << result_transf_filename << std::endl;

	return EXIT_SUCCESS;
}