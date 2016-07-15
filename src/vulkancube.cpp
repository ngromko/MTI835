/*
Gère la creation de cubes pour la scène
*/

#include "VulkanCube.h"

VulkanCube::VulkanCube(VkDevice mdevice, VulkanExampleBase *mexample,VkQueue queue, glm::vec3 halfSize, glm::mat4 startPos, bool burnable, float mass, uint32_t objectNumber, std::vector<BurningPoint>& points) : VulkanObject(mdevice,mexample,startPos,burnable)
{
    prepareVertices(halfSize,points, objectNumber);
    prepareIdex(queue);
    prepareRigidBody(halfSize, startPos[3], mass);
}

VulkanCube::~VulkanCube()
{
}

void VulkanCube::prepareVertices(glm::vec3 halfSize,std::vector<BurningPoint>& bPoints, uint32_t objectNumber){

    // Setup vertices
    std::vector<Vertex> vBuffer;

    // -Y
    Vertex v = { { halfSize.x,-halfSize.y, halfSize.z },{ 1.0, 1.0 }, { 0.0f, -1.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { -halfSize.x,-halfSize.y,-halfSize.z },{ 0.0, 0.0 },{ 0.0f, -1.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { halfSize.x,-halfSize.y,-halfSize.z },{ 1.0, 0.0 },{ 0.0f, -1.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { halfSize.x,-halfSize.y, halfSize.z },{ 1.0, 1.0 },{ 0.0f, -1.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { -halfSize.x,-halfSize.y, halfSize.z },{ 0.0, 1.0 },{ 0.0f, -1.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { -halfSize.x,-halfSize.y,-halfSize.z },{ 0.0, 0.0 },{ 0.0f, -1.0f, 0.0f }}; vBuffer.push_back(v);
    // +Y
     v = { { halfSize.x,halfSize.y, halfSize.z },{ 1.0, 1.0 },{ 0.0f, 1.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { halfSize.x,halfSize.y,-halfSize.z },{ 1.0, 0.0 },{ 0.0f, 1.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { -halfSize.x,halfSize.y,-halfSize.z },{ 0.0, 0.0 },{ 0.0f, 1.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { halfSize.x,halfSize.y, halfSize.z },{ 1.0, 1.0 },{ 0.0f, 1.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { -halfSize.x,halfSize.y,-halfSize.z },{ 0.0, 0.0 },{ 0.0f, 1.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { -halfSize.x,halfSize.y, halfSize.z },{ 0.0, 1.0 },{ 0.0f, 1.0f, 0.0f }}; vBuffer.push_back(v);
    // -X
     v = { { -halfSize.x,-halfSize.y,-halfSize.z },{ 0.0, 0.0 },{ -1.0f, 0.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { -halfSize.x,-halfSize.y, halfSize.z },{ 0.0, 1.0 },{ -1.0f, 0.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { -halfSize.x,halfSize.y, halfSize.z },{ 1.0, 1.0 },{ -1.0f, 0.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { -halfSize.x,-halfSize.y,-halfSize.z },{ 0.0, 0.0 },{ -1.0f, 0.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { -halfSize.x,halfSize.y, halfSize.z },{ 1.0, 1.0 },{ -1.0f, 0.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { -halfSize.x,halfSize.y,-halfSize.z },{ 1.0, 0.0 },{ -1.0f, 0.0f, 0.0f }}; vBuffer.push_back(v);
    // +X
     v = { { halfSize.x,halfSize.y, halfSize.z },{ 1.0, 1.0 },{ 1.0f, 0.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { halfSize.x,-halfSize.y,-halfSize.z },{ 0.0, 0.0 },{ 1.0f, 0.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { halfSize.x,halfSize.y,-halfSize.z },{ 1.0, 0.0 },{ 1.0f, 0.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { halfSize.x,-halfSize.y,-halfSize.z },{ 0.0, 0.0 },{ 1.0f, 0.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { halfSize.x,halfSize.y, halfSize.z },{ 1.0, 1.0 },{ 1.0f, 0.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { halfSize.x,-halfSize.y, halfSize.z },{ 0.0, 1.0 },{ 1.0f, 0.0f, 0.0f }}; vBuffer.push_back(v);
    // -Z
     v = { { halfSize.x,halfSize.y,-halfSize.z },{ 1.0, 1.0 },{ 0.0f, 0.0f, -1.0f }}; vBuffer.push_back(v);
     v = { { -halfSize.x,-halfSize.y,-halfSize.z },{ 0.0, 0.0 },{ 0.0f, 0.0f, -1.0f }}; vBuffer.push_back(v);
     v = { { -halfSize.x,halfSize.y,-halfSize.z },{ 0.0, 1.0 },{ 0.0f, 0.0f, -1.0f }}; vBuffer.push_back(v);
     v = { { halfSize.x,halfSize.y,-halfSize.z },{ 1.0, 1.0 },{ 0.0f, 0.0f, -1.0f }}; vBuffer.push_back(v);
     v = { { halfSize.x,-halfSize.y,-halfSize.z },{ 1.0, 0.0 },{ 0.0f, 0.0f, -1.0f }}; vBuffer.push_back(v);
     v = { { -halfSize.x,-halfSize.y,-halfSize.z },{ 0.0, 0.0 },{ 0.0f, 0.0f, -1.0f }}; vBuffer.push_back(v);
    // +Z
     v = { { -halfSize.x,halfSize.y, halfSize.z },{ 0.0, 1.0 },{ 0.0f, 0.0f, 1.0f }}; vBuffer.push_back(v);
     v = { { -halfSize.x,-halfSize.y, halfSize.z },{ 0.0, 0.0 },{ 0.0f, 0.0f, 1.0f }}; vBuffer.push_back(v);
     v = { { halfSize.x,-halfSize.y, halfSize.z },{ 1.0, 0.0 },{ 0.0f, 0.0f, 1.0f }}; vBuffer.push_back(v);
     v = { { halfSize.x,halfSize.y, halfSize.z },{ 1.0, 1.0 },{ 0.0f, 0.0f, 1.0f }}; vBuffer.push_back(v);
     v = { { -halfSize.x,halfSize.y, halfSize.z },{ 0.0, 1.0 },{ 0.0f, 0.0f, 1.0f }}; vBuffer.push_back(v);
     v = { { halfSize.x,-halfSize.y, halfSize.z },{ 1.0, 0.0 },{ 0.0f, 0.0f, 1.0f }}; vBuffer.push_back(v);

    float scalex = (halfSize.x-0.025)/halfSize.x;
    float scaley = (halfSize.y-0.025)/halfSize.y;
    float scalez = (halfSize.z-0.025)/halfSize.z;

    burnStart = bPoints.size();

    if(burnable){
        for(int i=0;i<36;i+=3){
            triangulate(vBuffer[i],vBuffer[i+1],vBuffer[i+2],bPoints);
        }
    }
    else{
        BurningPoint bp;
        for(int i=0;i<36;i++){
            bp.pos = glm::vec4(vBuffer[i].pos,0.0f);
            bp.basePos = glm::vec4(vBuffer[i].pos,1.0f);
            bp.baseNorm = bp.normal = glm::vec4(vBuffer[i].normal,0.0f);
            bp.uvCoord = vBuffer[i].uv;
            bp.nCount = 0;
            bp.life.x=-1.0f;
            bp.heat = 0.0f;
            bPoints.push_back(bp);
            indices.push_back(i);
        }
    }

    burnCount = bPoints.size()-burnStart;
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
        bPoints[i].nCount+=objectNumber<<16;
    }
}

void VulkanCube::prepareRigidBody(glm::vec3 size, glm::vec4 startPos, float mass){

    btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(size.x),btScalar(size.y),btScalar(size.z)));
    btVector3 localInertia(0,0,0);

    btTransform groundTransform;
    groundTransform.setIdentity();

    groundTransform.setOrigin(btVector3(startPos.x,startPos.y,startPos.z));

    VkObjectMotionState* myMotionState = new VkObjectMotionState(groundTransform,this);
    if(mass!=0){
        groundShape->calculateLocalInertia(mass,localInertia);
    }

    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
    rbody = new btRigidBody(rbInfo,true);
}
