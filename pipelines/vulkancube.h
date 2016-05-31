/*
* Vulkan Example - Animated gears using multiple uniform buffers
*
* See readme.md for details
*
* Copyright (C) 2015 by Sascha Willems - www.saschawillems.de
*
* This code is licensed under the MIT license (MIT) (http://opensource.org/licenses/MIT)
*/

#pragma once

#include <math.h>
#include <vector>

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/matrix_inverse.hpp>

#include "vulkan/vulkan.h"

#include "vulkantools.h"
#include "vulkanexamplebase.h"
#include "btBulletDynamicsCommon.h"

struct Vertex
{
	float pos[3];
	float normal[3];
    float uv[2];
	float color[3];
};

class VulkanCube
{
private:
	struct UBO
	{
		glm::mat4 projection;
        glm::mat4 model;
		glm::mat4 view;
	};

	VkDevice device;

	// Reference to example for getting memory types
	VulkanExampleBase *exampleBase;

	struct
	{
		VkBuffer buf;
		VkDeviceMemory mem;
	} vertexBuffer;

	UBO ubo;
	vkTools::UniformData uniformData;

    void prepareUniformBuffer(glm::vec3 startPos);
    void prepareVertices(float size);
    void prepareRigidBody(float size,glm::vec3 startPos,float mass);
    glm::mat4 VulkanCube::btmattoglm(btTransform mat);

    btRigidBody* rbody;
public:


	VkDescriptorSet descriptorSet;

	void draw(VkCommandBuffer cmdbuffer, VkPipelineLayout pipelineLayout);
    void updateUniformBuffer(glm::mat4 proj, glm::mat4 view);

	void setupDescriptorSet(VkDescriptorPool pool, VkDescriptorSetLayout descriptorSetLayout);

    VulkanCube(VkDevice device, VulkanExampleBase *example,float halfSize,glm::vec3 startPos,float mass);
    ~VulkanCube();

    btRigidBody* getRigidBody();

    void update();

};

