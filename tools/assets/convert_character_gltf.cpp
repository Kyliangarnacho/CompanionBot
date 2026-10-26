// Offline asset conversion only: borrow installed Assimp importer/exporter.
// This tool does not render images, synthesize meshes or alter animation data.
#include <assimp/Importer.hpp>
#include <assimp/Exporter.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/version.h>
#include <iostream>
int main(int argc, char **argv) {
  if (argc != 3) { std::cerr << "usage: convert_character_gltf source.dae destination.gltf\n"; return 2; }
  Assimp::Importer importer;
  const aiScene *scene = importer.ReadFile(argv[1], aiProcess_Triangulate);
  if (!scene) { std::cerr << importer.GetErrorString() << '\n'; return 1; }
  if (!scene->HasAnimations() || !scene->HasMeshes()) {
    std::cerr << "source must contain existing mesh and animation\n"; return 1;
  }
  Assimp::Exporter exporter;
  if (exporter.Export(scene, "gltf2", argv[2]) != AI_SUCCESS) {
    std::cerr << exporter.GetErrorString() << '\n'; return 1;
  }
  std::cout << "{\"assimp_version\":\"" << aiGetVersionMajor() << '.' << aiGetVersionMinor()
            << '.' << aiGetVersionPatch() << "\",\"source_meshes\":" << scene->mNumMeshes
            << ",\"source_animations\":" << scene->mNumAnimations << "}\n";
}
