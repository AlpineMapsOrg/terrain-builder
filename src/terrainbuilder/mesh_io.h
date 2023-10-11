#include <fmt/core.h>

#include <assimp/Exporter.hpp>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include <glm/glm.hpp>
#include <glm/vector_relational.hpp>

void save_mesh_as_dae(
    const std::vector<unsigned int> &indices,
    const std::vector<glm::dvec3>& positions,
    const std::vector<glm::dvec2>& uvs) {
    // Create a new scene
    aiScene scene;
    scene.mRootNode = new aiNode();

    // Create a mesh
    scene.mMeshes = new aiMesh *[1];
    scene.mNumMeshes = 1;

    // Set our mesh to the root node
    scene.mRootNode->mMeshes = new unsigned int[1];
    scene.mRootNode->mNumMeshes = 1;
    scene.mRootNode->mMeshes[0] = 0;

    // Build mesh
    auto mesh = scene.mMeshes[0] = new aiMesh();

    mesh->mNumVertices = positions.size();
    mesh->mVertices = new aiVector3D[mesh->mNumVertices];
    for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
        mesh->mVertices[i] = aiVector3D(positions[i].x, positions[i].y, positions[i].z);
    }

    mesh->mNumFaces = indices.size() / 3;
    mesh->mFaces = new aiFace[mesh->mNumFaces];
    for (unsigned int i = 0; i < mesh->mNumFaces; i++) {
        aiFace &face = mesh->mFaces[i];
        face.mNumIndices = 3;
        face.mIndices = new unsigned int[3];
        for (unsigned int j = 0; j < 3; j++) {
            face.mIndices[j] = indices[i * 3 + j];
        }
    }

    // Set UV coordinates
    mesh->mNumUVComponents[0] = uvs.size();
    mesh->mTextureCoords[0] = new aiVector3D[mesh->mNumUVComponents[0]];
    for (unsigned int i = 0; i < mesh->mNumUVComponents[0]; ++i) {
        mesh->mTextureCoords[0][i] = aiVector3D(uvs[i].x, uvs[i].y, 0.0f);
    }

    int width, height, nrChannels;
    unsigned char *textureData = stbi_load("/mnt/c/Users/Admin/Downloads/142994.jpeg", &width, &height, &nrChannels, 0);

    aiTexel *texels = new aiTexel[width * height];
    for (int i = 0; i < width * height; i++) {
        texels[i].b = textureData[i * 3];
        texels[i].g = textureData[i * 3 + 1];
        texels[i].r = textureData[i * 3 + 2];
        texels[i].a = 0xFF; // You can set the alpha channel if needed.
    }

    scene.mNumTextures = 1;
    scene.mTextures = new aiTexture *[1];
    scene.mTextures[0] = new aiTexture();
    scene.mTextures[0]->mWidth = 256;
    scene.mTextures[0]->mHeight = 256;
    scene.mTextures[0]->pcData = texels;
    memcpy(scene.mTextures[0]->achFormatHint, "jpg\0", 4);

    // Create a material
    scene.mMaterials = new aiMaterial *[1];
    scene.mMaterials[0] = new aiMaterial();
    scene.mMaterials[0]->AddProperty("a\0", 1, AI_MATKEY_TEXTURE_DIFFUSE(0));
    scene.mNumMaterials = 1;

    // Set our material
    scene.mMeshes[0]->mMaterialIndex = 0;


    // Export to file
    Assimp::Exporter exporter;
    auto format = exporter.GetExportFormatDescription(0);
    auto result=exporter.Export(&scene, format->id, "test.dae\0");
    // const char* binaryDaeFormatID = "db3";
    // exporter.Export(&scene, binaryDaeFormatID, "test.dae");

    std::cout << result << std::endl;
    auto a = AI_SUCCESS;
    std::cout << a << std::endl;

    if (result != aiReturn_SUCCESS) {
        // Print the error message
        const char *errorString = exporter.GetErrorString();
        std::cerr << "Export failed: " << errorString << std::endl;
    } else {
        std::cout << "Export successful" << std::endl;
    }
}
