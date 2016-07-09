/*
Gère la creation de cubes pour la scène
*/

#include "VulkanCube.h"

VulkanCube::VulkanCube(VkDevice mdevice, VulkanExampleBase *mexample,glm::vec3 halfSize,vkTools::VulkanTexture *eTexture, glm::mat4 startPos,float mass): VulkanObject(mdevice,mexample,eTexture)
{
    prepareVertices(halfSize,glm::vec3(1,1,1));
    prepareRigidBody(halfSize, startPos[3], mass);
    prepareUniformBuffer(startPos);
}

VulkanCube::VulkanCube(VkDevice mdevice, VulkanExampleBase *mexample,glm::vec3 halfSize,vkTools::VulkanTexture *eTexture, glm::mat4 startPos,float mass,std::vector<glm::vec3>& points) : VulkanObject(mdevice,mexample,eTexture)
{
    points = prepareVertices(halfSize,glm::vec3(1,1,1));
    prepareRigidBody(halfSize, startPos[3], mass);
    prepareUniformBuffer(startPos);
}

VulkanCube::~VulkanCube()
{
	vkDestroyBuffer(device, vertexBuffer.buf, nullptr);
	vkFreeMemory(device, vertexBuffer.mem, nullptr);
}

std::vector<glm::vec3> VulkanCube::prepareVertices(glm::vec3 halfSize,glm::vec3 color){

    // Setup vertices
    glm::vec3 point;
    std::vector<Vertex> vBuffer;

    // -Y
    Vertex v = { { halfSize.x,-halfSize.y, halfSize.z }, { color.x, color.y, color.z },{ 1.0, 1.0 }, { 0.0f, -1.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { -halfSize.x,-halfSize.y,-halfSize.z }, { color.x, color.y, color.z },{ 0.0, 0.0 },{ 0.0f, -1.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { halfSize.x,-halfSize.y,-halfSize.z }, { color.x, color.y, color.z },{ 1.0, 0.0 },{ 0.0f, -1.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { halfSize.x,-halfSize.y, halfSize.z }, { color.x, color.y, color.z },{ 1.0, 1.0 },{ 0.0f, -1.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { -halfSize.x,-halfSize.y, halfSize.z }, { color.x, color.y, color.z },{ 0.0, 1.0 },{ 0.0f, -1.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { -halfSize.x,-halfSize.y,-halfSize.z }, { color.x, color.y, color.z },{ 0.0, 0.0 },{ 0.0f, -1.0f, 0.0f }}; vBuffer.push_back(v);
    // +Y
     v = { { halfSize.x,halfSize.y, halfSize.z }, { color.x, color.y, color.z },{ 1.0, 1.0 },{ 0.0f, 1.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { halfSize.x,halfSize.y,-halfSize.z }, { color.x, color.y, color.z },{ 1.0, 0.0 },{ 0.0f, 1.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { -halfSize.x,halfSize.y,-halfSize.z }, { color.x, color.y, color.z },{ 0.0, 0.0 },{ 0.0f, 1.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { halfSize.x,halfSize.y, halfSize.z }, { color.x, color.y, color.z },{ 1.0, 1.0 },{ 0.0f, 1.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { -halfSize.x,halfSize.y,-halfSize.z }, { color.x, color.y, color.z },{ 0.0, 0.0 },{ 0.0f, 1.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { -halfSize.x,halfSize.y, halfSize.z }, { color.x, color.y, color.z },{ 0.0, 1.0 },{ 0.0f, 1.0f, 0.0f }}; vBuffer.push_back(v);
    // -X
     v = { { -halfSize.x,-halfSize.y,-halfSize.z }, { color.x, color.y, color.z },{ 0.0, 0.0 },{ -1.0f, 0.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { -halfSize.x,-halfSize.y, halfSize.z }, { color.x, color.y, color.z },{ 0.0, 1.0 },{ -1.0f, 0.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { -halfSize.x,halfSize.y, halfSize.z }, { color.x, color.y, color.z },{ 1.0, 1.0 },{ -1.0f, 0.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { -halfSize.x,-halfSize.y,-halfSize.z }, { color.x, color.y, color.z },{ 0.0, 0.0 },{ -1.0f, 0.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { -halfSize.x,halfSize.y, halfSize.z }, { color.x, color.y, color.z },{ 1.0, 1.0 },{ -1.0f, 0.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { -halfSize.x,halfSize.y,-halfSize.z }, { color.x, color.y, color.z },{ 1.0, 0.0 },{ -1.0f, 0.0f, 0.0f }}; vBuffer.push_back(v);
    // +X
     v = { { halfSize.x,halfSize.y, halfSize.z }, { color.x, color.y, color.z },{ 1.0, 1.0 },{ 1.0f, 0.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { halfSize.x,-halfSize.y,-halfSize.z }, { color.x, color.y, color.z },{ 0.0, 0.0 },{ 1.0f, 0.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { halfSize.x,halfSize.y,-halfSize.z }, { color.x, color.y, color.z },{ 1.0, 0.0 },{ 1.0f, 0.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { halfSize.x,-halfSize.y,-halfSize.z }, { color.x, color.y, color.z },{ 0.0, 0.0 },{ 1.0f, 0.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { halfSize.x,halfSize.y, halfSize.z }, { color.x, color.y, color.z },{ 1.0, 1.0 },{ 1.0f, 0.0f, 0.0f }}; vBuffer.push_back(v);
     v = { { halfSize.x,-halfSize.y, halfSize.z }, { color.x, color.y, color.z },{ 0.0, 1.0 },{ 1.0f, 0.0f, 0.0f }}; vBuffer.push_back(v);
    // -Z
     v = { { halfSize.x,halfSize.y,-halfSize.z }, { color.x, color.y, color.z },{ 1.0, 1.0 },{ 0.0f, 0.0f, -1.0f }}; vBuffer.push_back(v);
     v = { { -halfSize.x,-halfSize.y,-halfSize.z }, { color.x, color.y, color.z },{ 0.0, 0.0 },{ 0.0f, 0.0f, -1.0f }}; vBuffer.push_back(v);
     v = { { -halfSize.x,halfSize.y,-halfSize.z }, { color.x, color.y, color.z },{ 0.0, 1.0 },{ 0.0f, 0.0f, -1.0f }}; vBuffer.push_back(v);
     v = { { halfSize.x,halfSize.y,-halfSize.z }, { color.x, color.y, color.z },{ 1.0, 1.0 },{ 0.0f, 0.0f, -1.0f }}; vBuffer.push_back(v);
     v = { { halfSize.x,-halfSize.y,-halfSize.z }, { color.x, color.y, color.z },{ 1.0, 0.0 },{ 0.0f, 0.0f, -1.0f }}; vBuffer.push_back(v);
     v = { { -halfSize.x,-halfSize.y,-halfSize.z }, { color.x, color.y, color.z },{ 0.0, 0.0 },{ 0.0f, 0.0f, -1.0f }}; vBuffer.push_back(v);
    // +Z
     v = { { -halfSize.x,halfSize.y, halfSize.z }, { color.x, color.y, color.z },{ 0.0, 1.0 },{ 0.0f, 0.0f, 1.0f }}; vBuffer.push_back(v);
     v = { { -halfSize.x,-halfSize.y, halfSize.z }, { color.x, color.y, color.z },{ 0.0, 0.0 },{ 0.0f, 0.0f, 1.0f }}; vBuffer.push_back(v);
     v = { { halfSize.x,-halfSize.y, halfSize.z }, { color.x, color.y, color.z },{ 1.0, 0.0 },{ 0.0f, 0.0f, 1.0f }}; vBuffer.push_back(v);
     v = { { halfSize.x,halfSize.y, halfSize.z }, { color.x, color.y, color.z },{ 1.0, 1.0 },{ 0.0f, 0.0f, 1.0f }}; vBuffer.push_back(v);
     v = { { -halfSize.x,halfSize.y, halfSize.z }, { color.x, color.y, color.z },{ 0.0, 1.0 },{ 0.0f, 0.0f, 1.0f }}; vBuffer.push_back(v);
     v = { { halfSize.x,-halfSize.y, halfSize.z }, { color.x, color.y, color.z },{ 1.0, 0.0 },{ 0.0f, 0.0f, 1.0f }}; vBuffer.push_back(v);

    float scalex = (halfSize.x-0.025)/halfSize.x;
    float scaley = (halfSize.y-0.025)/halfSize.y;
    float scalez = (halfSize.z-0.025)/halfSize.z;
    std::vector<glm::vec3> result;
    for(int i=0;i<12;i++){
        for(int j=0;j<3;j++){
            point = glm::vec3(vBuffer.at(3*i+j).pos[0]*scalex,vBuffer.at(3*i+j).pos[1]*scaley,vBuffer.at(3*i+j).pos[2]*scalez);
            result.push_back(point);
        }
        result.push_back(glm::vec3(vBuffer.at(3*i).normal[0],vBuffer.at(3*i).normal[1],vBuffer.at(3*i).normal[2]));
    }
	int vertexBufferSize = vBuffer.size() * sizeof(Vertex);

	VkMemoryAllocateInfo memAlloc = vkTools::initializers::memoryAllocateInfo();
	VkMemoryRequirements memReqs;

	VkResult err;
	void *data;

	// Generate vertex buffer
	VkBufferCreateInfo vBufferInfo = vkTools::initializers::bufferCreateInfo(VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, vertexBufferSize);
	err = vkCreateBuffer(device, &vBufferInfo, nullptr, &vertexBuffer.buf);
	assert(!err);
	vkGetBufferMemoryRequirements(device, vertexBuffer.buf, &memReqs);
	memAlloc.allocationSize = memReqs.size;
	exampleBase->getMemoryType(memReqs.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT, &memAlloc.memoryTypeIndex);
	err = vkAllocateMemory(device, &memAlloc, nullptr, &vertexBuffer.mem);
	assert(!err);
	err = vkMapMemory(device, vertexBuffer.mem, 0, vertexBufferSize, 0, &data);
	assert(!err);
	memcpy(data, vBuffer.data(), vertexBufferSize);
	vkUnmapMemory(device, vertexBuffer.mem);
	err = vkBindBufferMemory(device, vertexBuffer.buf, vertexBuffer.mem, 0);
	assert(!err);
    return result;
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


void VulkanCube::draw(VkCommandBuffer cmdbuffer, VkPipelineLayout pipelineLayout)
{
	VkDeviceSize offsets[1] = { 0 };
    vkCmdBindDescriptorSets(cmdbuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &descriptorSet, 0, NULL);
	vkCmdBindVertexBuffers(cmdbuffer, 0, 1, &vertexBuffer.buf, offsets);
    vkCmdDraw(cmdbuffer, 36, 1, 0, 0);
}
