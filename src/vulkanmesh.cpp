/*
Gère la création d'objets à partir du fichiers externe.
*/

#include "VulkanMesh.h"

VulkanMesh::VulkanMesh(VkDevice mdevice, VulkanExampleBase *mexample,VkQueue queue, std::vector<unsigned int> Indices, bool burnable, glm::mat4 startModel, float mass, uint32_t objectNumber, uint32_t bPointStart, std::vector<BurningPoint> & bPoints): VulkanObject(mdevice,mexample,startModel,burnable)
{
    indices=Indices;
    burnStart = bPointStart;
    burnCount = bPoints.size()-burnStart;
    std::cout<< " mesh " << burnStart << " " << burnCount<< objectNumber<<std::endl;
    prepareRigidBody(bPoints, startModel, mass, objectNumber);
    prepareIdex(queue);
}

VulkanMesh::~VulkanMesh()
{
}

void VulkanMesh::prepareRigidBody(std::vector<BurningPoint>& bPoints, glm::mat4 startPos, float mass, uint32_t objectNumber){

    btConvexHullShape* groundShape = new btConvexHullShape();

    for(int i=burnStart;i<bPoints.size();i++){
        for(int j=burnStart;j<bPoints.size();j++){
            if(i!=j){
                glm::vec3 diff(bPoints[j].pos - bPoints[i].pos);
                float d = glm::dot(diff,diff);
                if(d<0.0121 && bPoints[j].pos.y >= bPoints[i].pos.y){
                    bPoints[i].neighboors[bPoints[i].nCount]=j;
                    bPoints[i].nCount++;
                }
                if(bPoints[i].nCount==10){
                    break;
                }
            }
        }
        bPoints[i].life.x=burnable?14.0f:-1.0f;
        bPoints[i].heat = 0.0f;
        bPoints[i].nCount+=objectNumber<<16;
        groundShape->addPoint(btVector3(bPoints[i].pos.x,bPoints[i].pos.y,bPoints[i].pos.z));
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
