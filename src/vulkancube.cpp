/*
* Vulkan Example - Animated gears using multiple uniform buffers
*
* See readme.md for details
*
* Copyright (C) 2015 by Sascha Willems - www.saschawillems.de
*
* This code is licensed under the MIT license (MIT) (http://opensource.org/licenses/MIT)
*/

#include "VulkanCube.h"

VulkanCube::VulkanCube(VkDevice mdevice, VulkanExampleBase *mexample,glm::vec3 halfSize,glm::vec3 color, glm::vec3 startPos,float mass): VulkanObject(mdevice,mexample)
{
    prepareVertices(halfSize,color);
    prepareRigidBody(halfSize, startPos, mass);
    prepareUniformBuffer(startPos);
}

VulkanCube::VulkanCube(VkDevice mdevice, VulkanExampleBase *mexample,glm::vec3 halfSize,glm::vec3 color, glm::vec3 startPos,float mass,std::vector<glm::vec3>& points) : VulkanObject(mdevice,mexample)
{
    points = prepareVertices(halfSize,color);
    prepareRigidBody(halfSize, startPos, mass);
    prepareUniformBuffer(startPos);
}

VulkanCube::~VulkanCube()
{
	vkDestroyBuffer(device, vertexBuffer.buf, nullptr);
	vkFreeMemory(device, vertexBuffer.mem, nullptr);
}

std::vector<glm::vec3> VulkanCube::prepareVertices(glm::vec3 halfSize,glm::vec3 color){

    // Setup vertices

    std::vector<Vertex> vBuffer;

        // -Y
        Vertex v = { { halfSize.x,-halfSize.y, halfSize.z }, { color.x, color.y, color.z },{ 1.0, 1.0 }, { 0.0f, 1.0f, 0.0f }}; vBuffer.push_back(v);
         v = { { -halfSize.x,-halfSize.y,-halfSize.z }, { color.x, color.y, color.z },{ 0.0, 0.0 },{ 0.0f, 1.0f, 0.0f }}; vBuffer.push_back(v);
         v = { { halfSize.x,-halfSize.y,-halfSize.z }, { color.x, color.y, color.z },{ 1.0, 0.0 },{ 0.0f, 1.0f, 0.0f }}; vBuffer.push_back(v);
         v = { { halfSize.x,-halfSize.y, halfSize.z }, { color.x, color.y, color.z },{ 1.0, 1.0 },{ 0.0f, 1.0f, 0.0f }}; vBuffer.push_back(v);
         v = { { -halfSize.x,-halfSize.y, halfSize.z }, { color.x, color.y, color.z },{ 0.0, 1.0 },{ 0.0f, 1.0f, 0.0f }}; vBuffer.push_back(v);
         v = { { -halfSize.x,-halfSize.y,-halfSize.z }, { color.x, color.y, color.z },{ 0.0, 0.0 },{ 0.0f, 1.0f, 0.0f }}; vBuffer.push_back(v);
        // +Y
         v = { { halfSize.x,halfSize.y, halfSize.z }, { color.x, color.y, color.z },{ 1.0, 1.0 },{ 0.0f, -1.0f, 0.0f }}; vBuffer.push_back(v);
         v = { { halfSize.x,halfSize.y,-halfSize.z }, { color.x, color.y, color.z },{ 1.0, 0.0 },{ 0.0f, -1.0f, 0.0f }}; vBuffer.push_back(v);
         v = { { -halfSize.x,halfSize.y,-halfSize.z }, { color.x, color.y, color.z },{ 0.0, 0.0 },{ 0.0f, -1.0f, 0.0f }}; vBuffer.push_back(v);
         v = { { halfSize.x,halfSize.y, halfSize.z }, { color.x, color.y, color.z },{ 1.0, 1.0 },{ 0.0f, -1.0f, 0.0f }}; vBuffer.push_back(v);
         v = { { -halfSize.x,halfSize.y,-halfSize.z }, { color.x, color.y, color.z },{ 0.0, 0.0 },{ 0.0f, -1.0f, 0.0f }}; vBuffer.push_back(v);
         v = { { -halfSize.x,halfSize.y, halfSize.z }, { color.x, color.y, color.z },{ 0.0, 1.0 },{ 0.0f, -1.0f, 0.0f }}; vBuffer.push_back(v);
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

    std::vector<glm::vec3> result;
    for(int i=0;i<12;i++){
        for(int j=0;j<3;j++){
            result.push_back(glm::vec3(vBuffer.at(3*i+j).pos[0],vBuffer.at(3*i+j).pos[1],vBuffer.at(3*i+j).pos[2]));
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

void VulkanCube::prepareRigidBody(glm::vec3 size, glm::vec3 startPos, float mass){

    btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(size.x),btScalar(size.y),btScalar(size.z)));
    btVector3 localInertia(0,0,0);

    btTransform groundTransform;
    groundTransform.setIdentity();

    groundTransform.setOrigin(btVector3(startPos.x,startPos.y,startPos.z));

    /*btMatrix3x3 bob = btMatrix3x3(uboVS.modelMatrix[0].x,uboVS.modelMatrix[1].x,uboVS.modelMatrix[2].x,
            uboVS.modelMatrix[0].y,uboVS.modelMatrix[1].y,uboVS.modelMatrix[2].y,
            uboVS.modelMatrix[0].z,uboVS.modelMatrix[1].z,uboVS.modelMatrix[2].z);

groundTransform.setBasis(bob);*/
    VkObjectMotionState* myMotionState = new VkObjectMotionState(groundTransform,this);
    if(mass!=0){
        groundShape->calculateLocalInertia(mass,localInertia);
    }

    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
    rbody = new btRigidBody(rbInfo,mass>0);
}


void VulkanCube::draw(VkCommandBuffer cmdbuffer, VkPipelineLayout pipelineLayout)
{
	VkDeviceSize offsets[1] = { 0 };
    vkCmdBindDescriptorSets(cmdbuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &descriptorSet, 0, NULL);
	vkCmdBindVertexBuffers(cmdbuffer, 0, 1, &vertexBuffer.buf, offsets);
    vkCmdDraw(cmdbuffer, 36, 1, 0, 0);
}

void VulkanCube::setupDescriptorSet(VkDescriptorPool pool, VkDescriptorSetLayout descriptorSetLayout, uint32_t offSet, uint8_t* pdata)
{
    offset = offSet;
    this->pBurn = pdata;
	VkDescriptorSetAllocateInfo allocInfo =
		vkTools::initializers::descriptorSetAllocateInfo(
			pool,
			&descriptorSetLayout,
			1);

	VkResult vkRes = vkAllocateDescriptorSets(device, &allocInfo, &descriptorSet);
	assert(!vkRes);
std::cout<<"bumbo"<<std::endl;
	// Binding 0 : Vertex shader uniform buffer
	VkWriteDescriptorSet writeDescriptorSet =
		vkTools::initializers::writeDescriptorSet(
			descriptorSet,
			VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
			0,
            &uniformData.descriptor);

	vkUpdateDescriptorSets(device, 1, &writeDescriptorSet, 0, NULL);
}

void VulkanCube::prepareUniformBuffer(glm::vec3 pos)
{
    ubo.model= glm::mat4();

    ubo.model = glm::translate(ubo.model,pos);

	VkResult err;
    exampleBase->createBuffer(
        VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
        sizeof(ubo),
        &ubo,
        &uniformData.buffer,
        &uniformData.memory,
        &uniformData.descriptor);
}

btRigidBody* VulkanCube::getRigidBody(){
    return rbody;
}
