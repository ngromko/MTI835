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

VulkanCube::VulkanCube(VkDevice device, VulkanExampleBase *example,float halfSize, glm::vec3 startPos,float mass)
{
    this->device = device;
    this->exampleBase = example;

    prepareVertices(halfSize);
    prepareRigidBody(halfSize, startPos, mass);
    prepareUniformBuffer(startPos);
}

VulkanCube::~VulkanCube()
{
	// Clean up vulkan resources
	vkDestroyBuffer(device, uniformData.buffer, nullptr);
	vkFreeMemory(device, uniformData.memory, nullptr);

	vkDestroyBuffer(device, vertexBuffer.buf, nullptr);
	vkFreeMemory(device, vertexBuffer.mem, nullptr);
}

void VulkanCube::prepareVertices(float d){

    // Setup vertices
#define colred { 1.0f, 0.0f, 0.0f }
#define colgreen { 0.0f, 1.0f, 0.0f }
#define colblue { 0.0f, 0.0f, 1.0f }
    #define colwhite { 1.0f, 1.0f, 1.0f }


    std::vector<Vertex> vBuffer;

        // -Y
        Vertex v = { { d,-d, d }, colred,{ 1.0, 1.0 }, { 0.0f, 1.0f, 0.0f }}; vBuffer.push_back(v);
         v = { { -d,-d,-d }, colred,{ 0.0, 0.0 },{ 0.0f, 1.0f, 0.0f }}; vBuffer.push_back(v);
         v = { { d,-d,-d }, colred,{ 1.0, 0.0 },{ 0.0f, 1.0f, 0.0f }}; vBuffer.push_back(v);
         v = { { d,-d, d }, colred,{ 1.0, 1.0 },{ 0.0f, 1.0f, 0.0f }}; vBuffer.push_back(v);
         v = { { -d,-d, d }, colred,{ 0.0, 1.0 },{ 0.0f, 1.0f, 0.0f }}; vBuffer.push_back(v);
         v = { { -d,-d,-d }, colred,{ 0.0, 0.0 },{ 0.0f, 1.0f, 0.0f }}; vBuffer.push_back(v);
        // +Y
         v = { { d, d, d }, colwhite,{ 1.0, 1.0 },{ 0.0f, -1.0f, 0.0f }}; vBuffer.push_back(v);
         v = { { d, d,-d }, colred,{ 1.0, 0.0 },{ 0.0f, -1.0f, 0.0f }}; vBuffer.push_back(v);
         v = { { -d, d,-d }, colred,{ 0.0, 0.0 },{ 0.0f, -1.0f, 0.0f }}; vBuffer.push_back(v);
         v = { { d, d, d }, colred,{ 1.0, 1.0 },{ 0.0f, -1.0f, 0.0f }}; vBuffer.push_back(v);
         v = { { -d, d,-d }, colred,{ 0.0, 0.0 },{ 0.0f, -1.0f, 0.0f }}; vBuffer.push_back(v);
         v = { { -d, d, d }, colred,{ 0.0, 1.0 },{ 0.0f, -1.0f, 0.0f }}; vBuffer.push_back(v);
        // -X
         v = { { -d,-d,-d }, colblue,{ 0.0, 0.0 },{ -1.0f, 0.0f, 0.0f }}; vBuffer.push_back(v);
         v = { { -d,-d, d }, colblue,{ 0.0, 1.0 },{ -1.0f, 0.0f, 0.0f }}; vBuffer.push_back(v);
         v = { { -d, d, d }, colblue,{ 1.0, 1.0 },{ -1.0f, 0.0f, 0.0f }}; vBuffer.push_back(v);
         v = { { -d,-d,-d }, colblue,{ 0.0, 0.0 },{ -1.0f, 0.0f, 0.0f }}; vBuffer.push_back(v);
         v = { { -d, d, d }, colblue,{ 1.0, 1.0 },{ -1.0f, 0.0f, 0.0f }}; vBuffer.push_back(v);
         v = { { -d, d,-d }, colblue,{ 1.0, 0.0 },{ -1.0f, 0.0f, 0.0f }}; vBuffer.push_back(v);
        // +X
         v = { { d, d, d }, colblue,{ 1.0, 1.0 },{ 1.0f, 0.0f, 0.0f }}; vBuffer.push_back(v);
         v = { { d,-d,-d }, colblue,{ 0.0, 0.0 },{ 1.0f, 0.0f, 0.0f }}; vBuffer.push_back(v);
         v = { { d, d,-d }, colblue,{ 1.0, 0.0 },{ 1.0f, 0.0f, 0.0f }}; vBuffer.push_back(v);
         v = { { d,-d,-d }, colblue,{ 0.0, 0.0 },{ 1.0f, 0.0f, 0.0f }}; vBuffer.push_back(v);
         v = { { d, d, d }, colblue,{ 1.0, 1.0 },{ 1.0f, 0.0f, 0.0f }}; vBuffer.push_back(v);
         v = { { d,-d, d }, colblue,{ 0.0, 1.0 },{ 1.0f, 0.0f, 0.0f }}; vBuffer.push_back(v);
        // -Z
         v = { { d, d,-d }, colwhite,{ 1.0, 1.0 },{ 0.0f, 0.0f, -1.0f }}; vBuffer.push_back(v);
         v = { { -d,-d,-d }, colgreen,{ 0.0, 0.0 },{ 0.0f, 0.0f, -1.0f }}; vBuffer.push_back(v);
         v = { { -d, d,-d }, colgreen,{ 0.0, 1.0 },{ 0.0f, 0.0f, -1.0f }}; vBuffer.push_back(v);
         v = { { d, d,-d }, colgreen,{ 1.0, 1.0 },{ 0.0f, 0.0f, -1.0f }}; vBuffer.push_back(v);
         v = { { d,-d,-d }, colgreen,{ 1.0, 0.0 },{ 0.0f, 0.0f, -1.0f }}; vBuffer.push_back(v);
         v = { { -d,-d,-d }, colgreen,{ 0.0, 0.0 },{ 0.0f, 0.0f, -1.0f }}; vBuffer.push_back(v);
        // +Z
         v = { { -d, d, d }, colgreen,{ 0.0, 1.0 },{ 0.0f, 0.0f, 1.0f }}; vBuffer.push_back(v);
         v = { { -d,-d, d }, colgreen,{ 0.0, 0.0 },{ 0.0f, 0.0f, 1.0f }}; vBuffer.push_back(v);
         v = { { d,-d, d }, colgreen,{ 1.0, 0.0 },{ 0.0f, 0.0f, 1.0f }}; vBuffer.push_back(v);
         v = { { d, d, d }, colgreen,{ 1.0, 1.0 },{ 0.0f, 0.0f, 1.0f }}; vBuffer.push_back(v);
         v = { { -d, d, d }, colgreen,{ 0.0, 1.0 },{ 0.0f, 0.0f, 1.0f }}; vBuffer.push_back(v);
         v = { { d,-d, d }, colgreen,{ 1.0, 0.0 },{ 0.0f, 0.0f, 1.0f }}; vBuffer.push_back(v);

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

    std::cout << " te " << vBuffer.data()[15].pos[2] <<" "<< vertexBufferSize <<" " << vBuffer.size() << " " << sizeof(Vertex) << std::endl;
}

void VulkanCube::prepareRigidBody(float size, glm::vec3 startPos, float mass){

    btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(size),btScalar(size),btScalar(size)));
    btVector3 localInertia(0,0,0);

    btTransform groundTransform;
    groundTransform.setIdentity();

    groundTransform.setOrigin(btVector3(startPos.x,startPos.y,startPos.z));

    /*btMatrix3x3 bob = btMatrix3x3(uboVS.modelMatrix[0].x,uboVS.modelMatrix[1].x,uboVS.modelMatrix[2].x,
            uboVS.modelMatrix[0].y,uboVS.modelMatrix[1].y,uboVS.modelMatrix[2].y,
            uboVS.modelMatrix[0].z,uboVS.modelMatrix[1].z,uboVS.modelMatrix[2].z);

groundTransform.setBasis(bob);*/

    btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);

    if(mass!=0){
        groundShape->calculateLocalInertia(mass,localInertia);
    }

    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
    rbody = new btRigidBody(rbInfo);
}


void VulkanCube::draw(VkCommandBuffer cmdbuffer, VkPipelineLayout pipelineLayout)
{
	VkDeviceSize offsets[1] = { 0 };
	vkCmdBindDescriptorSets(cmdbuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &descriptorSet, 0, NULL);
	vkCmdBindVertexBuffers(cmdbuffer, 0, 1, &vertexBuffer.buf, offsets);
    vkCmdDraw(cmdbuffer, 36, 1, 0, 0);
}

void VulkanCube::updateUniformBuffer(glm::mat4 perspective, glm::mat4 view)
{
	ubo.projection = perspective;

    ubo.view = view;

	uint8_t *pData;
	VkResult err = vkMapMemory(device, uniformData.memory, 0, sizeof(ubo), 0, (void **)&pData);
	assert(!err);
	memcpy(pData, &ubo, sizeof(ubo));
	vkUnmapMemory(device, uniformData.memory);
}

void VulkanCube::setupDescriptorSet(VkDescriptorPool pool, VkDescriptorSetLayout descriptorSetLayout)
{
	VkDescriptorSetAllocateInfo allocInfo =
		vkTools::initializers::descriptorSetAllocateInfo(
			pool,
			&descriptorSetLayout,
			1);

	VkResult vkRes = vkAllocateDescriptorSets(device, &allocInfo, &descriptorSet);
	assert(!vkRes);

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

	// Vertex shader uniform buffer block
	VkMemoryAllocateInfo allocInfo = vkTools::initializers::memoryAllocateInfo();
	VkMemoryRequirements memReqs;

	VkBufferCreateInfo bufferInfo = vkTools::initializers::bufferCreateInfo(
		VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
		sizeof(ubo));

	err = vkCreateBuffer(device, &bufferInfo, nullptr, &uniformData.buffer);
	assert(!err);
	vkGetBufferMemoryRequirements(device, uniformData.buffer, &memReqs);
	allocInfo.allocationSize = memReqs.size;
	exampleBase->getMemoryType(memReqs.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT, &allocInfo.memoryTypeIndex);
	err = vkAllocateMemory(device, &allocInfo, nullptr, &uniformData.memory);
	assert(!err);
	err = vkBindBufferMemory(device, uniformData.buffer, uniformData.memory, 0);
	assert(!err);

	uniformData.descriptor.buffer = uniformData.buffer;
	uniformData.descriptor.offset = 0;
	uniformData.descriptor.range = sizeof(ubo);
	uniformData.allocSize = allocInfo.allocationSize;
}

void VulkanCube::update(){


    /*btMatrix3x3 bob = btMatrix3x3(uboVS.modelMatrix[0].x,uboVS.modelMatrix[1].x,uboVS.modelMatrix[2].x,
            uboVS.modelMatrix[0].y,uboVS.modelMatrix[1].y,uboVS.modelMatrix[2].y,
            uboVS.modelMatrix[0].z,uboVS.modelMatrix[1].z,uboVS.modelMatrix[2].z);*/

    btTransform mat = rbody->getWorldTransform();
    ubo.model = btmattoglm(mat);
    //ubo.model=glm::translate(ubo.model,glm::vec3(0,-0.1,0));
    updateUniformBuffer(ubo.projection,ubo.view);
}

btRigidBody* VulkanCube::getRigidBody(){
    return rbody;
}

glm::mat4 VulkanCube::btmattoglm(btTransform mat){
    glm::mat4 temp = glm::mat4();

    btVector3 ori = mat.getOrigin();
    temp = glm::translate(temp,glm::vec3(ori.x(),ori.y(),ori.z()));

    btQuaternion qua = mat.getRotation();

    std::cout << "ang " << qua.getAngle()<< " " << qua.getAxis().x()<< " " << qua.getAxis().y()<< " " <<qua.getAxis().z()<< std::endl;


    temp = glm::rotate(temp,qua.getAngle(),glm::vec3(qua.getAxis().x(),qua.getAxis().y(),qua.getAxis().z()));


    return temp;
}

