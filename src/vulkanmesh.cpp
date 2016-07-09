/*
Gère la création d'objets à partir du fichiers externe.
*/

#include "VulkanMesh.h"

VulkanMesh::VulkanMesh(VkDevice mdevice, VulkanExampleBase *mexample, vkMeshLoader::MeshBuffer* eMeshBuffer, vkTools::VulkanTexture *eTexture, glm::mat4 atartModel, float mass, std::vector<glm::vec3> & bPoints): VulkanObject(mdevice,mexample,eTexture), meshBuffer(eMeshBuffer)
{
    //for(auto indice : meshBuffer->)
    prepareRigidBody(bPoints, atartModel, mass);
    prepareUniformBuffer(atartModel);
}

VulkanMesh::~VulkanMesh()
{
}

void VulkanMesh::prepareRigidBody(std::vector<glm::vec3> bPoints, glm::mat4 startPos, float mass){

    btConvexHullShape* groundShape = new btConvexHullShape();

    for(auto point :bPoints){
        groundShape->addPoint(btVector3(point.x,point.y,point.z));
    }
    btVector3 localInertia(0,0,0);

    btTransform groundTransform;
    groundTransform.setIdentity();
    btMatrix3x3 basis = btMatrix3x3(startPos[0].x,startPos[1].x,startPos[2].x,
                startPos[0].y,startPos[1].y,startPos[2].y,
                startPos[0].z,startPos[1].z,startPos[2].z);

    groundTransform.setBasis(basis);
    groundTransform.setOrigin(btVector3(startPos[3].x,startPos[3].y,startPos[3].z));

    VkObjectMotionState* myMotionState = new VkObjectMotionState(groundTransform,this);
    if(mass!=0){
        groundShape->calculateLocalInertia(mass,localInertia);
    }

    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
    rbody = new btRigidBody(rbInfo,true);
}


void VulkanMesh::draw(VkCommandBuffer cmdbuffer, VkPipelineLayout pipelineLayout)
{
    VkDeviceSize offsets[1] = { 0 };
    vkCmdBindDescriptorSets(cmdbuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &descriptorSet, 0, NULL);
    vkCmdBindVertexBuffers(cmdbuffer, 0, 1, &meshBuffer->vertices.buf, offsets);
    vkCmdBindIndexBuffer(cmdbuffer, meshBuffer->indices.buf, 0, VK_INDEX_TYPE_UINT32);
    vkCmdDrawIndexed(cmdbuffer, meshBuffer->indexCount, 1, 0, 0, 0);
}
