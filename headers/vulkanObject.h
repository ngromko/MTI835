#pragma once

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/matrix_inverse.hpp>

#include "vulkan/vulkan.h"
#include "vulkantools.h"
#include "vulkanexamplebase.h"

#include <Fade_2D.h>
using namespace GEOM_FADE2D;

#include "btBulletDynamicsCommon.h"


struct BurningPoint{
    glm::vec4 pos;
    glm::vec4 basePos;
    glm::vec4 normal;
    glm::vec4 baseNorm;
    glm::vec2 uvCoord;
    glm::vec2 life;
    uint32_t neighboors[10];
    uint32_t nCount;
    float heat;
};

class VulkanObject
{
protected:
    VkDevice device;
    VulkanExampleBase *exampleBase;
    uint8_t *pBurn;
    uint32_t offset;
    uint32_t burnStart;
    uint32_t burnCount;

    bool burnable;

    struct Vertex
    {
        glm::vec3 pos;
        glm::vec2 uv;
        glm::vec3 normal;
    };

    std::vector<uint32_t> indices;
    struct {
        VkDeviceMemory memory;
        VkBuffer buffer;
    } indicesBuffer;

    btRigidBody* rbody;

    virtual void triangulate(Vertex v1, Vertex v2, Vertex v3, std::vector<BurningPoint>& allPoints){
        Fade_2D dt;
        BurningPoint bp;
        glm::mat3 mat,matt;
        glm::vec3 u,v,w,a;
        float z;
        std::vector<Point2> vPoints;
        std::vector<Point2*> vPPoints;
        std::vector<Segment2> vSegments1;
        std::vector<Triangle2*> vTriangles2;
        std::vector<ConstraintGraph2*> vCG;

        uint32_t p1index;
        uint32_t p2index;
        uint32_t p3index;

        u = glm::normalize(v2.pos-v1.pos);
        w = v1.normal;
        v = glm::cross(u,w);

        mat = glm::mat3();

        mat[0]=u;
        mat[1]=v;
        mat[2]=w;

        matt =glm::transpose(mat);
        a = matt*v1.pos;
        vPoints.push_back(Point2(a.x,a.y));
        a = matt*v2.pos;
        vPoints.push_back(Point2(a.x,a.y));
        a = matt*v3.pos;
        vPoints.push_back(Point2(a.x,a.y));
        z=a.z;
        dt.insert(vPoints);
        for(int j=0;j<3;j++){
            Point2& p0(vPoints[j]);
            Point2& p1(vPoints[(j+1)%3]);
            vSegments1.push_back(Segment2(p0,p1));
        }

        ConstraintGraph2* pCG1=dt.createConstraint(vSegments1,CIS_CONSTRAINED_DELAUNAY);

        vCG.push_back(pCG1);

        Zone2* pZone=dt.createZone(vCG,ZL_GROW,vPoints[0]);
        dt.applyConstraintsAndZones();
        dt.refine(pZone,27,0.01,0.1,true);

        //pZone->getTriangles(vTriangles2);

        dt.getVertexPointers(vPPoints);
        dt.getTrianglePointers(vTriangles2);
        uint32_t start = allPoints.size()-burnStart;
        for(int i=0;i<vPPoints.size();i++){
            a =mat*glm::vec3(vPPoints[i]->x(),vPPoints[i]->y(),z);
            bp.pos = glm::vec4(a,0.0f);
            bp.basePos = glm::vec4(a,1.0f);
            bp.baseNorm = bp.normal = glm::vec4(v1.normal,0.0f);
            bp.uvCoord = getUv(v1,v2,v3,a);
            bp.nCount = 0;
            bp.life.x=14.0f;
            bp.heat = 0.0f;
            allPoints.push_back(bp);
        }
        for(int i=0;i<vTriangles2.size();i++){
            for(int j=0;j<vPPoints.size();j++){
                if(vTriangles2[i]->getCorner(0)==vPPoints[j]){
                   p1index = j+start;
                }
                else if(vTriangles2[i]->getCorner(1)==vPPoints[j]){
                    p2index = j+start;
                }
                else if(vTriangles2[i]->getCorner(2)==vPPoints[j]){
                    p3index = j+start;
                }
            }
            indices.push_back(p1index);
            indices.push_back(p2index);
            indices.push_back(p3index);
        }
        dt.deleteZone(pZone);
        vSegments1.clear();
        vPoints.clear();
        vCG.clear();
    }

    void prepareIdex(VkQueue queue){

        uint32_t storageBufferSize = indices.size()*sizeof(uint32_t);
        std::cout << "indice size "<< storageBufferSize<<std::endl;
        struct {
            VkDeviceMemory memory;
            VkBuffer buffer;
        } stagingBuffer;
        exampleBase->createBuffer(
            VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT,
            storageBufferSize,
            indices.data(),
            &stagingBuffer.buffer,
            &stagingBuffer.memory);

        exampleBase->createBuffer(
            VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
            storageBufferSize,
            nullptr,
            &indicesBuffer.buffer,
            &indicesBuffer.memory);

        // Copy to staging buffer
        VkCommandBuffer copyCmd = exampleBase->createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY, true);

        VkBufferCopy copyRegion = {};
        copyRegion.size = storageBufferSize;
        vkCmdCopyBuffer(
            copyCmd,
            stagingBuffer.buffer,
            indicesBuffer.buffer,
            1,
            &copyRegion);

        exampleBase->flushCommandBuffer(copyCmd, queue, true);

        vkFreeMemory(device, stagingBuffer.memory, nullptr);
        vkDestroyBuffer(device, stagingBuffer.buffer, nullptr);
    }

public:
    virtual void updateModel(glm::mat4 model){
        memcpy(pBurn+offset, &model, sizeof(model));
    }

    VulkanObject(VkDevice mdevice, VulkanExampleBase *mexample, glm::mat4 startModel, bool burn) : device(mdevice), exampleBase(mexample), burnable(burn), model(startModel){

    }

    ~VulkanObject(){
    }

    btRigidBody* getRigidBody(){
        return rbody;
    }

    void setupMemory(uint32_t offSet, uint8_t* pdata)
    {
        offset = offSet;
        this->pBurn = pdata;

        updateModel(model);
    }

    void draw(VkCommandBuffer cmdbuffer, VkBuffer* vertexBuffer)
    {
        VkDeviceSize offsets[1] = { 0 };
        vkCmdBindVertexBuffers(cmdbuffer, 0, 1, vertexBuffer, offsets);
        vkCmdBindIndexBuffer(cmdbuffer, indicesBuffer.buffer, 0, VK_INDEX_TYPE_UINT32);
        vkCmdDrawIndexed(cmdbuffer, indices.size(), 1, 0, burnStart, 0);
    }

private:
    glm::mat4 model;
    glm::vec2 getUv(Vertex v1, Vertex v2, Vertex v3, glm::vec3 pos){
        //http://answers.unity3d.com/questions/383804/calculate-uv-coordinates-of-3d-point-on-plane-of-m.html
        // calculate vectors from point f to vertices v1.pos, v2.pos and v3.pos:
        glm::vec3 f1 = v1.pos-pos;
        glm::vec3 f2 = v2.pos-pos;
        glm::vec3 f3 = v3.pos-pos;

        // calculate the areas and factors (order of parameters doesn't matter):
        float a = glm::length(glm::cross(v1.pos-v2.pos, v1.pos-v3.pos)); // main triangle area a
        float a1 = glm::length(glm::cross(f2, f3)) / a; // v1.pos's triangle area / a
        float a2 = glm::length(glm::cross(f3, f1)) / a; // v2.pos's triangle area / a
        float a3 = glm::length(glm::cross(f1, f2)) / a; // v3.pos's triangle area / a
        // find the uv corresponding to point f (uv1/uv2/uv3 are associated to v1.pos/v2.pos/v3.pos):
        return v1.uv * a1 + v2.uv * a2 + v3.uv * a3;
    }
};
